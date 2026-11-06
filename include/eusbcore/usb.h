/* SPDX-License-Identifier: GPL-2.0-or-later */
/* eusbcore: the native USB stack - host controller interface, the xHCI,
 * EHCI, OHCI and UHCI drivers, devices and the enumeration core (root
 * ports, hubs). Polled; built on OpenASPI's native host services
 * (openaspi/native/host.h). Class drivers (mass storage in EUSBASPI, HID in
 * openinput) use usb_control() and usb_transfer() on the devices the core
 * found. */

#ifndef EUSBCORE_USB_H
#define EUSBCORE_USB_H

#include "openaspi/types.h"

/* speeds, numbered as xHCI port speed IDs */
#define USB_SPEED_FULL  1u
#define USB_SPEED_LOW   2u
#define USB_SPEED_HIGH  3u
#define USB_SPEED_SUPER 4u

/* transfer results */
#define USB_OK      0
#define USB_STALL   1
#define USB_TIMEOUT 2
#define USB_ERROR   3 /* transaction, babble, buffer or controller error */

/* endpoint types (bmAttributes & 3) */
#define USB_EP_CONTROL 0u
#define USB_EP_ISOCH   1u
#define USB_EP_BULK    2u
#define USB_EP_INTR    3u

#define USB_MAX_DEVICES 32
#define USB_MAX_EP      16

struct usb_hc;

struct usb_ep {
    oa_u16 mps;      /* 0: not configured */
    oa_u8 type;      /* USB_EP_* */
    oa_u8 interval;  /* bInterval */
    oa_u8 toggle;    /* data toggle, for UHCI/OHCI/EHCI */
};

struct usb_dev {
    struct usb_hc *hc;
    struct usb_dev *parent;  /* hub, NULL on a root port */
    oa_u8 port;              /* port on the parent (1-based) or root port */
    oa_u8 root_port;         /* root port the tree hangs on (1-based) */
    oa_u8 speed;
    oa_u8 addr;              /* USB address, 0 until addressed */
    oa_u8 tier;              /* 0 on a root port */
    oa_u32 route;            /* xHCI route string */
    struct usb_dev *tt_hub;  /* high-speed hub with the TT for LS/FS devices */
    oa_u8 tt_port;
    oa_u8 is_hub, hub_ports;
    oa_u8 config;            /* bConfigurationValue */
    oa_u16 vendor, product;
    oa_u8 dev_class;
    struct usb_ep in[USB_MAX_EP], out[USB_MAX_EP]; /* in[0]/out[0]: EP0 */
    void *hcpriv;
    oa_u8 config_desc[512];  /* the whole configuration descriptor */
    oa_u16 config_len;
};

struct usb_hc_ops {
    /* root hub */
    unsigned (*ports)(struct usb_hc *hc);
    /* Reset root port `port` (1-based) if a device is connected; 0 and the
     * speed when it is enabled, -1 if nothing is there. */
    int (*port_reset)(struct usb_hc *hc, unsigned port, oa_u8 *speed);
    /* Give the new device (address 0, EP0 set up from its speed) an
     * address; 0 on success. */
    int (*address)(struct usb_hc *hc, struct usb_dev *dev);
    /* EP0 max packet size changed (from the device descriptor). */
    int (*update_ep0)(struct usb_hc *hc, struct usb_dev *dev);
    /* The endpoints in dev->in/out are set after SET_CONFIGURATION (xHCI
     * configures them, others need nothing). */
    int (*configure)(struct usb_hc *hc, struct usb_dev *dev);
    /* Control transfer on EP0; *actual: data bytes moved. */
    int (*control)(struct usb_hc *hc, struct usb_dev *dev, const oa_u8 setup[8], void *data,
                   oa_u16 len, oa_u16 *actual, oa_u32 timeout_ms);
    /* Bulk or interrupt transfer on endpoint `ep` (bit 7: IN), at most
     * hc->max_xfer bytes. A short IN packet ends it early. */
    int (*transfer)(struct usb_hc *hc, struct usb_dev *dev, oa_u8 ep, void *data, oa_u32 len,
                    oa_u32 *actual, oa_u32 timeout_ms);
    /* The endpoint halted and was cleared on the device: reset it on the
     * controller too (xHCI); the data toggle starts at DATA0 again. */
    void (*clear_halt)(struct usb_hc *hc, struct usb_dev *dev, oa_u8 ep);
    void (*release)(struct usb_hc *hc, struct usb_dev *dev);
};

struct usb_hc {
    const struct usb_hc_ops *ops;
    void *priv;
    char name[32];
    oa_u32 max_xfer;         /* bytes per transfer() call */
    oa_u8 next_addr;
    struct usb_hc *next;
};

/* Register an initialized host controller with the core. */
void usb_hc_register(struct usb_hc *hc);

/* Enumerate the root ports of every registered controller and the hubs
 * behind them. Returns the number of devices found. */
int usb_enumerate(void);

unsigned usb_device_count(void);
struct usb_dev *usb_device(unsigned i);

/* Standard requests through the controller. */
int usb_control(struct usb_dev *dev, oa_u8 type, oa_u8 req, oa_u16 value, oa_u16 index,
                void *data, oa_u16 len, oa_u16 *actual);
int usb_transfer(struct usb_dev *dev, oa_u8 ep, void *data, oa_u32 len, oa_u32 *actual,
                 oa_u32 timeout_ms);
/* CLEAR_FEATURE(ENDPOINT_HALT) and the controller side of it. */
int usb_clear_halt(struct usb_dev *dev, oa_u8 ep);

/* Walk the configuration descriptor: the next descriptor of `type` after
 * offset *pos (start with 0). Returns a pointer or NULL. */
const oa_u8 *usb_find_desc(const struct usb_dev *dev, oa_u8 type, oa_u16 *pos);

/* Set an address for controllers without an Address Device command
 * (UHCI, OHCI, EHCI): SET_ADDRESS on address 0. */
int usb_set_address(struct usb_hc *hc, struct usb_dev *dev);

/* PCI class/subclass of USB controllers and the programming interfaces. */
#define USB_PCI_CLASS 0x0C03u
#define USB_PCI_UHCI  0x00u
#define USB_PCI_OHCI  0x10u
#define USB_PCI_EHCI  0x20u
#define USB_PCI_XHCI  0x30u

/* Take over the controller at PCI address `pci` and register it with the
 * core; 0 on success, -1 if it is not usable. */
int usb_add_xhci(oa_u32 pci);
int usb_add_ehci(oa_u32 pci);
int usb_add_ohci(oa_u32 pci);
int usb_add_uhci(oa_u32 pci);

/* Any USB controller by its programming interface. EHCI controllers are to
 * be added before their companion UHCI/OHCI controllers. */
int usb_add(oa_u32 pci);

#endif
