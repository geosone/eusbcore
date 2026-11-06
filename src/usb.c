/* SPDX-License-Identifier: GPL-2.0-or-later */
/* USB enumeration core: root ports, hubs, descriptors (see usb.h).
 *
 * As in EUSBASPI.SYS: every connected port is reset, the device gets an
 * address, its descriptors are read and its first configuration is set.
 * Hubs are powered and their ports enumerated, up to five tiers; USB 3
 * hubs are not handled yet. Devices plugged in later are not seen. */

#include <string.h>

#include "eusbcore/usb.h"
#include "openaspi/native/host.h"

#define DESC_DEVICE 1u
#define DESC_CONFIG 2u
#define DESC_ENDPOINT 5u
#define DESC_HUB 0x29u

#define REQ_GET_STATUS  0u
#define REQ_CLEAR_FEAT  1u
#define REQ_SET_FEAT    3u
#define REQ_SET_ADDRESS 5u
#define REQ_GET_DESC    6u
#define REQ_SET_CONFIG  9u

#define HUB_PORT_CONNECTION  0u
#define HUB_PORT_ENABLE      1u
#define HUB_PORT_RESET       4u
#define HUB_PORT_POWER       8u
#define HUB_C_PORT_CONNECTION 16u
#define HUB_C_PORT_RESET     20u

static struct usb_hc *hcs;
static struct usb_dev devs[USB_MAX_DEVICES];
static unsigned ndevs;

void usb_hc_register(struct usb_hc *hc)
{
    struct usb_hc **p = &hcs;

    while (*p) {
        p = &(*p)->next;
    }
    hc->next = 0;
    hc->next_addr = 1;
    *p = hc;
}

unsigned usb_device_count(void)
{
    return ndevs;
}

struct usb_dev *usb_device(unsigned i)
{
    return i < ndevs ? &devs[i] : 0;
}

int usb_control(struct usb_dev *dev, oa_u8 type, oa_u8 req, oa_u16 value, oa_u16 index,
                void *data, oa_u16 len, oa_u16 *actual)
{
    oa_u8 setup[8];
    oa_u16 dummy;

    setup[0] = type;
    setup[1] = req;
    setup[2] = (oa_u8)value;
    setup[3] = (oa_u8)(value >> 8);
    setup[4] = (oa_u8)index;
    setup[5] = (oa_u8)(index >> 8);
    setup[6] = (oa_u8)len;
    setup[7] = (oa_u8)(len >> 8);
    return dev->hc->ops->control(dev->hc, dev, setup, data, len, actual ? actual : &dummy, 5000);
}

int usb_transfer(struct usb_dev *dev, oa_u8 ep, void *data, oa_u32 len, oa_u32 *actual,
                 oa_u32 timeout_ms)
{
    return dev->hc->ops->transfer(dev->hc, dev, ep, data, len, actual, timeout_ms);
}

int usb_intr_open(struct usb_intr *pipe, struct usb_dev *dev, oa_u8 ep, oa_u16 len)
{
    struct usb_hc *hc = dev->hc;
    const struct usb_ep *e = &dev->in[ep & 15u];

    if (!hc->ops->intr_open || !(ep & 0x80u) || e->mps == 0u || e->type != USB_EP_INTR) {
        return -1;
    }
    pipe->dev = dev;
    pipe->ep = ep;
    pipe->len = len < e->mps ? len : e->mps;
    pipe->hcpriv = 0;
    return hc->ops->intr_open(hc, pipe);
}

int usb_intr_poll(struct usb_intr *pipe, void *buf, oa_u16 *actual)
{
    struct usb_hc *hc = pipe->dev->hc;

    *actual = 0;
    return hc->ops->intr_poll(hc, pipe, buf, actual);
}

int usb_clear_halt(struct usb_dev *dev, oa_u8 ep)
{
    int r = usb_control(dev, 0x02u, REQ_CLEAR_FEAT, 0, ep, 0, 0, 0);

    if (dev->hc->ops->clear_halt) {
        dev->hc->ops->clear_halt(dev->hc, dev, ep);
    }
    if (ep & 0x80u) {
        dev->in[ep & 15u].toggle = 0;
    } else {
        dev->out[ep & 15u].toggle = 0;
    }
    return r;
}

int usb_set_address(struct usb_hc *hc, struct usb_dev *dev)
{
    oa_u8 addr = hc->next_addr;
    int r;

    if (addr > 127u) {
        return -1;
    }
    r = usb_control(dev, 0x00u, REQ_SET_ADDRESS, addr, 0, 0, 0, 0);
    if (r != USB_OK) {
        return -1;
    }
    oa_host_delay_us(2000); /* SET_ADDRESS recovery */
    hc->next_addr++;
    dev->addr = addr;
    return 0;
}

const oa_u8 *usb_find_desc(const struct usb_dev *dev, oa_u8 type, oa_u16 *pos)
{
    oa_u16 p = *pos;
    const oa_u8 *d;

    if (p != 0u) { /* skip the one found last time */
        p = (oa_u16)(p + dev->config_desc[p]);
    }
    while (p + 2u <= dev->config_len) {
        d = dev->config_desc + p;
        if (d[0] < 2u || p + d[0] > dev->config_len) {
            return 0;
        }
        if (d[1] == type) {
            *pos = p;
            return d;
        }
        p = (oa_u16)(p + d[0]);
    }
    return 0;
}

static int enumerate_device(struct usb_hc *hc, struct usb_dev *parent, unsigned port,
                            oa_u8 speed);

/* Reset hub port `port`; 0 and the speed when a device is enabled. */
static int hub_port_reset(struct usb_dev *hub, unsigned port, oa_u8 *speed)
{
    oa_u8 st[4];
    oa_u16 status = 0, change = 0, got;
    unsigned i;

    if (usb_control(hub, 0xA3u, REQ_GET_STATUS, 0, (oa_u16)port, st, 4, &got) != USB_OK ||
        got < 4u) {
        return -1;
    }
    status = (oa_u16)(st[0] | (st[1] << 8));
    if (!(status & 1u)) {
        return -1; /* nothing connected */
    }
    usb_control(hub, 0x23u, REQ_CLEAR_FEAT, HUB_C_PORT_CONNECTION, (oa_u16)port, 0, 0, 0);
    if (usb_control(hub, 0x23u, REQ_SET_FEAT, HUB_PORT_RESET, (oa_u16)port, 0, 0, 0) != USB_OK) {
        return -1;
    }
    for (i = 0; i < 50u; i++) {
        oa_host_delay_us(10000);
        if (usb_control(hub, 0xA3u, REQ_GET_STATUS, 0, (oa_u16)port, st, 4, &got) != USB_OK) {
            return -1;
        }
        status = (oa_u16)(st[0] | (st[1] << 8));
        change = (oa_u16)(st[2] | (st[3] << 8));
        if ((change & 0x10u) || !(status & 0x10u)) { /* C_PORT_RESET, or reset over */
            break;
        }
    }
    usb_control(hub, 0x23u, REQ_CLEAR_FEAT, HUB_C_PORT_RESET, (oa_u16)port, 0, 0, 0);
    if (!(status & 2u)) {
        return -1; /* not enabled */
    }
    oa_host_delay_us(10000); /* reset recovery */
    if (status & 0x0200u) {
        *speed = USB_SPEED_LOW;
    } else if (status & 0x0400u) {
        *speed = USB_SPEED_HIGH;
    } else {
        *speed = USB_SPEED_FULL;
    }
    return 0;
}

static void hub_scan(struct usb_dev *hub)
{
    oa_u8 desc[16], speed;
    oa_u16 got;
    unsigned i, nports, delay;

    if (hub->speed == USB_SPEED_SUPER || hub->tier >= 4u) {
        return; /* USB 3 hubs: not yet; at most five tiers */
    }
    if (usb_control(hub, 0xA0u, REQ_GET_DESC, DESC_HUB << 8, 0, desc, sizeof(desc), &got) !=
            USB_OK ||
        got < 7u) {
        return;
    }
    nports = desc[2];
    delay = desc[5] * 2u; /* power on to power good, ms */
    hub->hub_ports = (oa_u8)nports;
    if (hub->hc->ops->configure) {
        hub->hc->ops->configure(hub->hc, hub); /* xHCI: the hub bit and port count */
    }
    for (i = 1; i <= nports; i++) {
        usb_control(hub, 0x23u, REQ_SET_FEAT, HUB_PORT_POWER, (oa_u16)i, 0, 0, 0);
    }
    oa_host_delay_us((delay < 100u ? 100u : delay) * 1000u);
    for (i = 1; i <= nports && i <= 15u; i++) {
        if (hub_port_reset(hub, i, &speed) == 0) {
            enumerate_device(hub->hc, hub, i, speed);
        }
    }
}

static void parse_endpoints(struct usb_dev *dev)
{
    const oa_u8 *d;
    oa_u16 pos = 0;
    struct usb_ep *ep;

    while ((d = usb_find_desc(dev, DESC_ENDPOINT, &pos)) != 0) {
        if (d[0] < 7u || (d[2] & 15u) == 0u) {
            continue;
        }
        ep = (d[2] & 0x80u) ? &dev->in[d[2] & 15u] : &dev->out[d[2] & 15u];
        ep->mps = (oa_u16)((d[4] | (d[5] << 8)) & 0x7FFu);
        ep->type = (oa_u8)(d[3] & 3u);
        ep->interval = d[6];
        ep->toggle = 0;
    }
}

static int enumerate_device(struct usb_hc *hc, struct usb_dev *parent, unsigned port,
                            oa_u8 speed)
{
    struct usb_dev *dev;
    oa_u8 desc[18];
    oa_u16 got, total;

    if (ndevs == USB_MAX_DEVICES) {
        return -1;
    }
    dev = &devs[ndevs];
    memset(dev, 0, sizeof(*dev));
    dev->hc = hc;
    dev->parent = parent;
    dev->port = (oa_u8)port;
    dev->speed = speed;
    if (parent) {
        dev->tier = (oa_u8)(parent->tier + 1u);
        dev->root_port = parent->root_port;
        dev->route = parent->route | ((oa_u32)(port > 15u ? 15u : port) << (4u * parent->tier));
        if (speed != USB_SPEED_HIGH && speed != USB_SPEED_SUPER) {
            if (parent->speed == USB_SPEED_HIGH) {
                dev->tt_hub = parent;
                dev->tt_port = (oa_u8)port;
            } else {
                dev->tt_hub = parent->tt_hub;
                dev->tt_port = parent->tt_port;
            }
        }
    } else {
        dev->root_port = (oa_u8)port;
    }
    dev->out[0].mps = dev->in[0].mps =
        speed == USB_SPEED_LOW ? 8u : speed == USB_SPEED_SUPER ? 512u : 64u;
    dev->out[0].type = dev->in[0].type = USB_EP_CONTROL;

    if (hc->ops->address(hc, dev) != 0) {
        oa_host_log(OA_LOG_WARN, "USB: %s port %u: no address\n", hc->name, port);
        return -1;
    }
    /* the first 8 bytes tell EP0's packet size */
    if (usb_control(dev, 0x80u, REQ_GET_DESC, DESC_DEVICE << 8, 0, desc, 8, &got) != USB_OK ||
        got < 8u) {
        goto fail;
    }
    if (speed != USB_SPEED_SUPER && desc[7] != dev->out[0].mps && desc[7] >= 8u) {
        dev->out[0].mps = dev->in[0].mps = desc[7];
        if (hc->ops->update_ep0) {
            hc->ops->update_ep0(hc, dev);
        }
    }
    if (usb_control(dev, 0x80u, REQ_GET_DESC, DESC_DEVICE << 8, 0, desc, 18, &got) != USB_OK ||
        got < 18u) {
        goto fail;
    }
    dev->dev_class = desc[4];
    dev->vendor = (oa_u16)(desc[8] | (desc[9] << 8));
    dev->product = (oa_u16)(desc[10] | (desc[11] << 8));

    if (usb_control(dev, 0x80u, REQ_GET_DESC, DESC_CONFIG << 8, 0, dev->config_desc, 9, &got) !=
            USB_OK ||
        got < 9u) {
        goto fail;
    }
    total = (oa_u16)(dev->config_desc[2] | (dev->config_desc[3] << 8));
    if (total > sizeof(dev->config_desc)) {
        total = sizeof(dev->config_desc);
    }
    if (usb_control(dev, 0x80u, REQ_GET_DESC, DESC_CONFIG << 8, 0, dev->config_desc, total,
                    &got) != USB_OK) {
        goto fail;
    }
    dev->config_len = got;
    dev->config = dev->config_desc[5];
    parse_endpoints(dev);
    if (usb_control(dev, 0x00u, REQ_SET_CONFIG, dev->config, 0, 0, 0, 0) != USB_OK) {
        goto fail;
    }
    dev->is_hub = dev->dev_class == 9u;
    if (hc->ops->configure && !dev->is_hub && hc->ops->configure(hc, dev) != 0) {
        goto fail;
    }
    ndevs++;
    oa_host_log(OA_LOG_INFO, "USB: %s port %u%s: %04x:%04x class %02x, %s speed\n", hc->name,
                port, parent ? " (hub)" : "", dev->vendor, dev->product, dev->dev_class,
                speed == USB_SPEED_LOW ? "low" : speed == USB_SPEED_FULL ? "full"
                : speed == USB_SPEED_HIGH ? "high" : "super");
    if (dev->is_hub) {
        hub_scan(dev);
    }
    return 0;
fail:
    oa_host_log(OA_LOG_WARN, "USB: %s port %u: device does not answer\n", hc->name, port);
    if (hc->ops->release) {
        hc->ops->release(hc, dev);
    }
    return -1;
}

int usb_enumerate(void)
{
    struct usb_hc *hc;
    unsigned port, n;
    oa_u8 speed;

    for (hc = hcs; hc; hc = hc->next) {
        n = hc->ops->ports(hc);
        for (port = 1; port <= n; port++) {
            if (hc->ops->port_reset(hc, port, &speed) == 0) {
                enumerate_device(hc, 0, port, speed);
            }
        }
    }
    return (int)ndevs;
}

/* Any USB controller by its programming interface (usb.h). */
int usb_add(oa_u32 pci)
{
    oa_u32 cc = oa_host_pci_read32(pci, 0x08) >> 8;

    if ((cc >> 8) != USB_PCI_CLASS) {
        return -1;
    }
    switch (cc & 0xFFu) {
    case USB_PCI_XHCI:
        return usb_add_xhci(pci);
    case USB_PCI_EHCI:
        return usb_add_ehci(pci);
    case USB_PCI_OHCI:
        return usb_add_ohci(pci);
    case USB_PCI_UHCI:
        return usb_add_uhci(pci);
    }
    return -1;
}
