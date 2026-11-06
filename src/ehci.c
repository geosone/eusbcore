/* SPDX-License-Identifier: GPL-2.0-or-later */
/* EHCI host controller for eusbcore, the native USB stack (polled).
 *
 * As EUSBASPI.SYS's ehci_init: BIOS handoff (USB legacy support in PCI
 * configuration space, SMIs off), reset, async schedule only. Each
 * endpoint gets a queue head that stays in the async ring; a transfer is
 * a chain of qTDs hung into it, polled until the controller is done.
 * Full- and low-speed devices on a root port go to the companion
 * controller (port owner); behind a high-speed hub they use split
 * transactions. Interrupt endpoints run on the async schedule too, which
 * is fine for high-speed devices (CBI floppies are full speed). */

#include <string.h>

#include "eusbcore/usb.h"
#include "openaspi/native/host.h"

#define MAX_CONTROLLERS 4
#define MAX_QH          64
#define NQTD            8u
#define BOUNCE_SIZE     65536u
#define QTD_BYTES       16384u

#define OP_USBCMD  0x00u
#define OP_USBSTS  0x04u
#define OP_USBINTR 0x08u
#define OP_CTRLDS  0x10u
#define OP_FRINDEX 0x0Cu
#define OP_PERIODIC 0x14u
#define OP_ASYNC   0x18u
#define OP_CONFIG  0x40u
#define OP_PORTSC  0x44u

#define CMD_RS    0x01u
#define CMD_RESET 0x02u
#define CMD_PSE   0x10u
#define CMD_ASE   0x20u
#define CMD_IAAD  0x40u
#define STS_IAA   0x20u
#define STS_HALT  0x1000u
#define STS_ASS   0x8000u

#define PORT_CCS   0x0001u
#define PORT_CSC   0x0002u
#define PORT_PE    0x0004u
#define PORT_PEC   0x0008u
#define PORT_OCC   0x0020u
#define PORT_PR    0x0100u
#define PORT_LS    0x0C00u
#define PORT_PP    0x1000u
#define PORT_OWNER 0x2000u
#define PORT_WC    (PORT_CSC | PORT_PEC | PORT_OCC)

#define TOK_ACTIVE 0x80u
#define TOK_HALTED 0x40u
#define TOK_ERRORS 0x7Cu /* halted, buffer, babble, transaction, missed */
#define PID_OUT    0u
#define PID_IN     1u
#define PID_SETUP  2u

struct qh {
    oa_u32 link, ep_char, ep_caps, cur;
    oa_u32 next, alt, token, buf[5], buf_hi[5];
    oa_u32 pad[3];  /* 64 bytes */
    /* driver side, not seen by the controller */
};

struct qtd {
    oa_u32 next, alt, token, buf[5], buf_hi[5];
    oa_u32 pad[3];  /* 64 bytes */
};

struct pipe {
    struct usb_dev *dev;
    oa_u8 ep;
    volatile struct qh *qh;
    oa_u64 qh_bus;
};

struct ehc {
    struct usb_hc hc;
    volatile oa_u8 *op;
    unsigned nports;
    volatile struct qh *head;
    oa_u64 head_bus;
    volatile struct qtd *td;      /* NQTD */
    oa_u64 td_bus;
    oa_u8 *bounce;
    oa_u64 bounce_bus;
    struct pipe pipes[MAX_QH];
    unsigned npipes;
    oa_u32 *frames;               /* periodic frame list, 1024 entries */
};

/* An interrupt IN pipe: a queue head in the periodic schedule (polled in
 * every frame) with one qTD that is re-armed after each report. */
struct eintr {
    volatile struct qh *qh;
    oa_u64 qh_bus;
    volatile struct qtd *td;
    oa_u64 td_bus;
    oa_u8 *buf;
    oa_u64 buf_bus;
    oa_u16 len;
};

#define MAX_INTR 16
static struct eintr eintrs[MAX_INTR];
static unsigned neintrs;

static struct ehc ehcs[MAX_CONTROLLERS];
static unsigned nehcs;

static oa_u32 rd(struct ehc *e, unsigned off)
{
    return *(volatile oa_u32 *)(e->op + off);
}

static void wr(struct ehc *e, unsigned off, oa_u32 v)
{
    *(volatile oa_u32 *)(e->op + off) = v;
}

static int wait_bits(struct ehc *e, unsigned off, oa_u32 mask, oa_u32 value, oa_u32 ms)
{
    oa_u64 end = oa_host_time_us() + (oa_u64)ms * 1000u;

    while ((rd(e, off) & mask) != value) {
        if (oa_host_time_us() > end) {
            return -1;
        }
        oa_host_delay_us(50);
    }
    return 0;
}

static unsigned e_ports(struct usb_hc *hc)
{
    return ((struct ehc *)hc->priv)->nports;
}

static int e_port_reset(struct usb_hc *hc, unsigned port, oa_u8 *speed)
{
    struct ehc *e = hc->priv;
    unsigned off = OP_PORTSC + 4u * (port - 1u);
    oa_u32 v = rd(e, off);

    if (!(v & PORT_CCS)) {
        return -1;
    }
    if ((v & PORT_LS) == 0x0400u) { /* K state: low speed, to the companion */
        wr(e, off, (v & ~PORT_WC) | PORT_OWNER);
        return -1;
    }
    wr(e, off, (v & ~(PORT_WC | PORT_PE)) | PORT_PR);
    oa_host_delay_us(50000);
    wr(e, off, rd(e, off) & ~(PORT_WC | PORT_PR));
    if (wait_bits(e, off, PORT_PR, 0, 10) < 0) {
        return -1;
    }
    oa_host_delay_us(10000);
    v = rd(e, off);
    wr(e, off, v); /* clear the change bits */
    if (!(v & PORT_PE)) { /* full speed: the companion takes it */
        wr(e, off, (rd(e, off) & ~PORT_WC) | PORT_OWNER);
        return -1;
    }
    *speed = USB_SPEED_HIGH;
    return 0;
}

static int e_address(struct usb_hc *hc, struct usb_dev *dev)
{
    return usb_set_address(hc, dev);
}

/* The queue head of an endpoint, created and linked in on first use. */
static struct pipe *pipe_of(struct ehc *e, struct usb_dev *d, oa_u8 ep)
{
    struct pipe *p;
    unsigned i, eps, mps;
    oa_u32 ch, caps;

    for (i = 0; i < e->npipes; i++) {
        if (e->pipes[i].dev == d && e->pipes[i].ep == ep) {
            p = &e->pipes[i];
            goto update;
        }
    }
    if (e->npipes == MAX_QH) {
        return 0;
    }
    p = &e->pipes[e->npipes];
    p->qh = oa_host_dma_alloc(64, 64, 1, &p->qh_bus);
    if (p->qh == 0) {
        return 0;
    }
    e->npipes++;
    p->dev = d;
    p->ep = ep;
    memset((void *)p->qh, 0, 64);
    p->qh->next = 1u;
    p->qh->alt = 1u;
    /* into the ring after the head */
    p->qh->link = e->head->link;
    oa_host_mb();
    e->head->link = (oa_u32)p->qh_bus | 2u;
update:
    eps = d->speed == USB_SPEED_HIGH ? 2u : d->speed == USB_SPEED_LOW ? 1u : 0u;
    mps = (ep & 15u) == 0u ? d->out[0].mps : (ep & 0x80u) ? d->in[ep & 15u].mps : d->out[ep & 15u].mps;
    ch = d->addr | ((oa_u32)(ep & 15u) << 8) | (eps << 12) | ((oa_u32)mps << 16) | (4ul << 28);
    if ((ep & 15u) == 0u) {
        ch |= 1ul << 14; /* toggle from the qTD */
        if (eps != 2u) {
            ch |= 1ul << 27; /* control endpoint, not high speed */
        }
    }
    caps = 1ul << 30;
    if (eps != 2u && d->tt_hub) {
        caps |= ((oa_u32)d->tt_hub->addr << 16) | ((oa_u32)d->tt_port << 23);
    }
    p->qh->ep_char = ch;
    p->qh->ep_caps = caps;
    return p;
}

/* Takes a queue head out of the async ring while the controller may still
 * work on it (a timed-out transfer): unlink it, ring the Interrupt on Async
 * Advance doorbell and wait until the controller has let go of it (EHCI
 * 1.0, 4.8.2), clear the overlay, link it in again after the head. */
static void quiesce(struct ehc *e, struct pipe *p)
{
    oa_u32 me = (oa_u32)p->qh_bus | 2u;
    volatile struct qh *prev = e->head;
    unsigned i;

    for (i = 0; i < e->npipes && prev->link != me; i++) {
        prev = e->pipes[i].qh;
    }
    if (prev->link != me) {
        return; /* not linked */
    }
    prev->link = p->qh->link;
    oa_host_mb();
    wr(e, OP_USBSTS, STS_IAA);
    wr(e, OP_USBCMD, rd(e, OP_USBCMD) | CMD_IAAD);
    wait_bits(e, OP_USBSTS, STS_IAA, STS_IAA, 20);
    wr(e, OP_USBSTS, STS_IAA);
    p->qh->next = 1u;
    p->qh->token = p->qh->token & (1ul << 31); /* idle, keep the toggle */
    p->qh->cur = 0;
    p->qh->link = e->head->link;
    oa_host_mb();
    e->head->link = me;
    oa_host_mb();
}

/* Run the qTD chain td[0..n-1] on the pipe. Done when every qTD is
 * inactive, or when one ended short and the next is still active (the
 * controller left the chain through the alternate link). */
static int run(struct ehc *e, struct pipe *p, unsigned n, oa_u32 timeout_ms)
{
    oa_u64 end = oa_host_time_us() + (oa_u64)timeout_ms * 1000u;
    oa_u32 tok;
    unsigned i;

    p->qh->token = p->qh->token & (1ul << 31); /* overlay idle, keep the toggle */
    p->qh->cur = 0;
    oa_host_mb();
    p->qh->next = (oa_u32)e->td_bus;
    oa_host_mb();
    for (;;) {
        for (i = 0; i < n; i++) {
            tok = e->td[i].token;
            if (tok & TOK_ACTIVE) {
                if (i > 0u && ((e->td[i - 1u].token >> 16) & 0x7FFFu) != 0u &&
                    !(p->qh->token & TOK_ACTIVE) && (p->qh->next & 1u)) {
                    goto done; /* short packet */
                }
                break;
            }
            if (tok & TOK_ERRORS) {
                goto error;
            }
        }
        if (i == n) {
            goto done;
        }
        if (oa_host_time_us() > end) {
            goto error;
        }
    }
done:
    p->qh->next = 1u;
    oa_host_mb();
    return USB_OK;
error:
    if (i >= n) {
        i = n - 1u;
    }
    tok = e->td[i].token;
    if (tok & TOK_ACTIVE) {
        /* timed out: the controller may still be on it */
        quiesce(e, p);
        return USB_TIMEOUT;
    }
    /* halted: the controller leaves the queue head alone */
    p->qh->next = 1u;
    p->qh->token = 0;
    oa_host_mb();
    if ((tok & TOK_HALTED) && !(tok & 0x3Cu)) {
        return USB_STALL;
    }
    return USB_ERROR;
}

static void fill(struct ehc *e, unsigned i, oa_u32 pid, oa_u64 bus, oa_u32 len, oa_u32 toggle,
                 int last)
{
    volatile struct qtd *t = &e->td[i];
    unsigned k;

    t->next = last ? 1u : (oa_u32)(e->td_bus + 64u * (i + 1u));
    t->alt = 1u;
    t->token = (toggle << 31) | (len << 16) | (3ul << 10) | (pid << 8) | TOK_ACTIVE;
    for (k = 0; k < 5u; k++) {
        oa_u64 b = k == 0u ? bus : ((bus & ~(oa_u64)0xFFFu) + 4096u * k);
        t->buf[k] = (oa_u32)b;
        t->buf_hi[k] = (oa_u32)(b >> 32);
    }
}

static oa_u32 done_bytes(struct ehc *e, unsigned i, oa_u32 len)
{
    oa_u32 left = (e->td[i].token >> 16) & 0x7FFFu;
    return left <= len ? len - left : 0u;
}

static int e_control(struct usb_hc *hc, struct usb_dev *d, const oa_u8 setup[8], void *data,
                     oa_u16 len, oa_u16 *actual, oa_u32 timeout_ms)
{
    struct ehc *e = hc->priv;
    struct pipe *p = pipe_of(e, d, 0);
    int in = (setup[0] & 0x80u) != 0, r;
    unsigned n = 0;

    *actual = 0;
    if (p == 0 || len > QTD_BYTES) {
        return USB_ERROR;
    }
    memcpy(e->bounce, setup, 8);
    if (!in && len) {
        memcpy(e->bounce + 4096, data, len);
    }
    fill(e, n++, PID_SETUP, e->bounce_bus, 8, 0, 0);
    if (len) {
        fill(e, n++, in ? PID_IN : PID_OUT, e->bounce_bus + 4096u, len, 1, 0);
        e->td[1].alt = (oa_u32)(e->td_bus + 64u * 2u); /* short: on to the status stage */
    }
    fill(e, n++, (len && in) ? PID_OUT : PID_IN, 0, 0, 1, 1);
    oa_host_mb();
    r = run(e, p, n, timeout_ms ? timeout_ms : 5000u);
    if (r != USB_OK) {
        return r;
    }
    if (len) {
        *actual = (oa_u16)done_bytes(e, 1, len);
        if (in && *actual) {
            memcpy(data, e->bounce + 4096, *actual);
        }
    }
    return USB_OK;
}

static int e_transfer(struct usb_hc *hc, struct usb_dev *d, oa_u8 ep, void *data, oa_u32 len,
                      oa_u32 *actual, oa_u32 timeout_ms)
{
    struct ehc *e = hc->priv;
    struct pipe *p = pipe_of(e, d, ep);
    struct usb_ep *uep = (ep & 0x80u) ? &d->in[ep & 15u] : &d->out[ep & 15u];
    int in = (ep & 0x80u) != 0, r;
    unsigned n = 0, i;
    oa_u32 left = len, chunk, off = 0, moved = 0, mps = uep->mps ? uep->mps : 64u, pkts = 0;

    *actual = 0;
    if (p == 0 || len > BOUNCE_SIZE) {
        return USB_ERROR;
    }
    if (!in && len) {
        memcpy(e->bounce, data, len);
    }
    do {
        chunk = left < QTD_BYTES ? left : QTD_BYTES;
        fill(e, n, in ? PID_IN : PID_OUT, e->bounce_bus + off, chunk, 0, 0);
        n++;
        off += chunk;
        left -= chunk;
    } while (left && n < NQTD);
    e->td[n - 1u].next = 1u;
    for (i = 0; i < n; i++) {
        e->td[i].alt = 1u;
    }
    /* the queue head keeps the data toggle (DTC = 0) */
    oa_host_mb();
    r = run(e, p, n, timeout_ms ? timeout_ms : 15000u);
    for (i = 0; i < n; i++) {
        oa_u32 want = i + 1u < n ? QTD_BYTES : len - QTD_BYTES * i;
        oa_u32 got = done_bytes(e, i, want);
        if (e->td[i].token & TOK_ACTIVE) {
            break;
        }
        moved += got;
        pkts += (got + mps - 1u) / mps;
        if (got < want) {
            break;
        }
    }
    (void)pkts;
    *actual = moved;
    if (in && moved) {
        memcpy(data, e->bounce, moved);
    }
    return r;
}

static void e_clear_halt(struct usb_hc *hc, struct usb_dev *d, oa_u8 ep)
{
    struct ehc *e = hc->priv;
    struct pipe *p = pipe_of(e, d, ep);

    if (p) {
        p->qh->next = 1u;
        p->qh->token = 0; /* not halted, toggle DATA0 */
        oa_host_mb();
    }
}

static void intr_arm(struct eintr *p)
{
    unsigned k;

    p->td->next = 1u;
    p->td->alt = 1u;
    for (k = 0; k < 5u; k++) {
        p->td->buf[k] = k == 0u ? (oa_u32)p->buf_bus : 0u;
        p->td->buf_hi[k] = k == 0u ? (oa_u32)(p->buf_bus >> 32) : 0u;
    }
    p->td->token = ((oa_u32)p->len << 16) | (3ul << 10) | (PID_IN << 8) | TOK_ACTIVE;
    oa_host_mb();
    p->qh->next = (oa_u32)p->td_bus;
    oa_host_mb();
}

static int e_intr_open(struct usb_hc *hc, struct usb_intr *pipe)
{
    struct ehc *e = hc->priv;
    struct usb_dev *d = pipe->dev;
    struct eintr *p;
    unsigned eps, i;
    oa_u32 ch, caps, head;

    if (neintrs == MAX_INTR || e->frames == 0) {
        return -1;
    }
    p = &eintrs[neintrs];
    p->qh = oa_host_dma_alloc(64, 64, 1, &p->qh_bus);
    p->td = oa_host_dma_alloc(64, 64, 1, &p->td_bus);
    p->buf = oa_host_dma_alloc(1024, 64, 1, &p->buf_bus);
    if (p->qh == 0 || p->td == 0 || p->buf == 0) {
        return -1;
    }
    neintrs++;
    p->len = pipe->len;
    eps = d->speed == USB_SPEED_HIGH ? 2u : d->speed == USB_SPEED_LOW ? 1u : 0u;
    ch = d->addr | ((oa_u32)(pipe->ep & 15u) << 8) | (eps << 12) |
         ((oa_u32)d->in[pipe->ep & 15u].mps << 16);
    caps = (1ul << 30) | 0x01u;            /* one packet, start in microframe 0 */
    if (eps != 2u) {
        caps |= 0x1Cu << 8;                /* complete splits in microframes 2-4 */
        if (d->tt_hub) {
            caps |= ((oa_u32)d->tt_hub->addr << 16) | ((oa_u32)d->tt_port << 23);
        }
    }
    memset((void *)p->qh, 0, 64);
    memset((void *)p->td, 0, 64);
    p->qh->ep_char = ch;
    p->qh->ep_caps = caps;
    p->qh->alt = 1u;
    p->qh->next = 1u;
    head = e->frames[0];
    p->qh->link = head;                    /* in front of the others */
    intr_arm(p);
    for (i = 0; i < 1024u; i++) {
        e->frames[i] = (oa_u32)p->qh_bus | 2u;
    }
    oa_host_mb();
    pipe->hcpriv = p;
    return 0;
}

static int e_intr_poll(struct usb_hc *hc, struct usb_intr *pipe, void *buf, oa_u16 *actual)
{
    struct eintr *p = pipe->hcpriv;
    oa_u32 tok = p->td->token, left;

    (void)hc;
    if (tok & TOK_ACTIVE) {
        return 0;
    }
    oa_host_mb();
    if (tok & TOK_ERRORS) {
        p->qh->token = 0; /* not halted, DATA0 */
        intr_arm(p);
        return (tok & TOK_HALTED) && !(tok & 0x3Cu) ? -USB_STALL : -USB_ERROR;
    }
    left = (tok >> 16) & 0x7FFFu;
    *actual = (oa_u16)(left < p->len ? p->len - left : 0u);
    memcpy(buf, p->buf, *actual);
    intr_arm(p);
    return 1;
}

static const struct usb_hc_ops ehci_ops = {
    e_ports, e_port_reset, e_address, 0, 0, e_control, e_transfer, e_clear_halt, 0,
    e_intr_open, e_intr_poll
};

static void hex(char *out, oa_u32 v, unsigned digits)
{
    while (digits--) {
        out[digits] = "0123456789abcdef"[v & 15u];
        v >>= 4;
    }
}

int usb_add_ehci(oa_u32 pci)
{
    struct ehc *e;
    volatile oa_u8 *cap;
    oa_u32 bar, cmd, hcs, hcc, eecp, v;
    unsigned i;
    oa_u64 end;

    if (nehcs == MAX_CONTROLLERS ||
        (oa_host_pci_read32(pci, 0x08) >> 8) != ((oa_u32)USB_PCI_CLASS << 8 | USB_PCI_EHCI)) {
        return -1;
    }
    bar = oa_host_pci_read32(pci, 0x10) & 0xFFFFFF00ul;
    if (bar == 0u) {
        return -1;
    }
    cmd = oa_host_pci_read32(pci, 0x04);
    oa_host_pci_write32(pci, 0x04, (cmd & 0xFFFFu) | 0x0006u);
    e = &ehcs[nehcs];
    memset(e, 0, sizeof(*e));
    cap = oa_host_map_mmio(bar, 0x400);
    if (cap == 0) {
        return -1;
    }
    e->op = cap + *(volatile oa_u8 *)cap;
    hcs = *(volatile oa_u32 *)(cap + 4);
    hcc = *(volatile oa_u32 *)(cap + 8);
    e->nports = hcs & 0xFu;

    /* BIOS handoff */
    eecp = (hcc >> 8) & 0xFFu;
    if (eecp >= 0x40u) {
        v = oa_host_pci_read32(pci, eecp);
        if ((v & 0xFFu) == 1u) {
            if (v & (1ul << 16)) {
                oa_host_pci_write32(pci, eecp, v | (1ul << 24));
                end = oa_host_time_us() + 1000000u;
                while ((oa_host_pci_read32(pci, eecp) & (1ul << 16)) && oa_host_time_us() < end) {
                    oa_host_delay_us(1000);
                }
                oa_host_pci_write32(pci, eecp, (oa_host_pci_read32(pci, eecp) & ~(1ul << 16)) | (1ul << 24));
            }
            oa_host_pci_write32(pci, eecp + 4u, 0xE0000000ul); /* SMIs off, status cleared */
        }
    }

    wr(e, OP_USBCMD, rd(e, OP_USBCMD) & ~CMD_RS);
    wait_bits(e, OP_USBSTS, STS_HALT, STS_HALT, 20);
    wr(e, OP_USBCMD, CMD_RESET);
    if (wait_bits(e, OP_USBCMD, CMD_RESET, 0, 250) < 0) {
        return -1;
    }
    e->head = oa_host_dma_alloc(64, 64, 1, &e->head_bus);
    e->td = oa_host_dma_alloc(64u * NQTD, 64, 1, &e->td_bus);
    e->bounce = oa_host_dma_alloc(BOUNCE_SIZE, 4096, 1, &e->bounce_bus);
    if (e->head == 0 || e->td == 0 || e->bounce == 0) {
        return -1;
    }
    memset((void *)e->head, 0, 64);
    e->head->link = (oa_u32)e->head_bus | 2u;
    e->head->ep_char = 1ul << 15; /* head of the reclamation list */
    e->head->next = 1u;
    e->head->alt = 1u;
    e->head->token = TOK_HALTED;
    wr(e, OP_USBINTR, 0);
    wr(e, OP_CTRLDS, 0);
    wr(e, OP_ASYNC, (oa_u32)e->head_bus);
    {
        oa_u64 fb;
        e->frames = oa_host_dma_alloc(4096, 4096, 1, &fb);
        if (e->frames) {
            for (i = 0; i < 1024u; i++) {
                e->frames[i] = 1u;
            }
            wr(e, OP_FRINDEX, 0);
            wr(e, OP_PERIODIC, (oa_u32)fb);
        }
    }
    oa_host_mb();
    wr(e, OP_USBCMD, (8ul << 16) | CMD_ASE | (e->frames ? CMD_PSE : 0u) | CMD_RS);
    wait_bits(e, OP_USBSTS, STS_HALT, 0, 20);
    wr(e, OP_CONFIG, 1); /* ports to us */
    if (hcs & 0x10u) { /* port power control */
        for (i = 0; i < e->nports; i++) {
            v = rd(e, OP_PORTSC + 4u * i);
            wr(e, OP_PORTSC + 4u * i, (v & ~PORT_WC) | PORT_PP);
        }
    }
    oa_host_delay_us(100000);

    memcpy(e->hc.name, "EHCI 0000:00:00.0", 18);
    hex(e->hc.name + 5, pci >> 16, 4);
    hex(e->hc.name + 10, (pci >> 8) & 0xFFu, 2);
    hex(e->hc.name + 13, (pci >> 3) & 0x1Fu, 2);
    e->hc.name[16] = (char)('0' + (pci & 7u));
    e->hc.ops = &ehci_ops;
    e->hc.priv = e;
    e->hc.max_xfer = QTD_BYTES * 4u; /* 64 KiB in four qTDs */
    nehcs++;
    usb_hc_register(&e->hc);
    return 0;
}
