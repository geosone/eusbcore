/* SPDX-License-Identifier: GPL-2.0-or-later */
/* xHCI host controller for eusbcore, the native USB stack (polled).
 *
 * As EUSBASPI.SYS's xhci_init: BIOS handoff through the USB legacy support
 * capability (SMIs off), controller reset, device context array with the
 * scratchpad buffers the controller asks for, one command ring and one
 * event ring segment, no interrupts. USB 2 ports are reset, USB 3 ports
 * enable themselves (a link that does not reach U0 gets a warm reset).
 * Each device has a slot with a transfer ring per endpoint; data goes
 * through a bounce buffer, one TRB per transfer. */

#include <string.h>

#include "eusbcore/usb.h"
#include "openaspi/native/host.h"

#define MAX_CONTROLLERS 4
#define MAX_SLOTS       32
#define RING_TRBS       64u   /* per transfer ring, last one is the link */
#define CMD_TRBS        64u
#define EVT_TRBS        256u
#define PAGE            4096u
#define BOUNCE_SIZE     65536u

/* capability registers */
#define CAP_LENGTH   0x00u
#define CAP_HCS1     0x04u
#define CAP_HCS2     0x08u
#define CAP_HCC1     0x10u
#define CAP_DBOFF    0x14u
#define CAP_RTSOFF   0x18u
/* operational registers */
#define OP_USBCMD    0x00u
#define OP_USBSTS    0x04u
#define OP_PAGESIZE  0x08u
#define OP_CRCR      0x18u
#define OP_DCBAAP    0x30u
#define OP_CONFIG    0x38u
#define OP_PORTSC    0x400u
/* interrupter 0 */
#define IR_IMAN      0x20u
#define IR_IMOD      0x24u
#define IR_ERSTSZ    0x28u
#define IR_ERSTBA    0x30u
#define IR_ERDP      0x38u

#define CMD_RS    0x01u
#define CMD_HCRST 0x02u
#define CMD_INTE  0x04u /* interrupter enable (host system interrupt enable) */
#define STS_HCH   0x01u
#define STS_CNR   0x800u

#define PORT_CCS  (1ul << 0)
#define PORT_PED  (1ul << 1)
#define PORT_PR   (1ul << 4)
#define PORT_PP   (1ul << 9)
#define PORT_WRC  (1ul << 19)
#define PORT_PRC  (1ul << 21)
#define PORT_WPR  (1ul << 31)
#define PORT_KEEP ((1ul << 9) | (3ul << 14) | (7ul << 25)) /* PP, PIC, wake bits */
#define PORT_CHANGES 0x00FE0000ul

/* TRB types */
#define TRB_NORMAL      1u
#define TRB_SETUP       2u
#define TRB_DATA        3u
#define TRB_STATUS      4u
#define TRB_LINK        6u
#define TRB_ENABLE_SLOT 9u
#define TRB_DISABLE_SLOT 10u
#define TRB_ADDRESS     11u
#define TRB_CONFIGURE   12u
#define TRB_EVALUATE    13u
#define TRB_RESET_EP    14u
#define TRB_STOP_EP     15u
#define TRB_SET_DEQ     16u
#define TRB_EV_TRANSFER 32u
#define TRB_EV_COMMAND  33u

#define TRB_CYCLE (1ul << 0)
#define TRB_TC    (1ul << 1) /* link: toggle cycle */
#define TRB_ISP   (1ul << 2)
#define TRB_IOC   (1ul << 5)
#define TRB_IDT   (1ul << 6)
#define TRB_DIR_IN (1ul << 16)

#define CC_SUCCESS 1u
#define CC_STALL   6u
#define CC_SHORT   13u

struct trb {
    oa_u32 p0, p1, status, control;
};

struct ring {
    volatile struct trb *trb;
    oa_u64 bus;
    unsigned enq;
    oa_u32 cycle;
};

struct slot {
    oa_u8 id;
    struct usb_dev *dev;
    oa_u8 *out_ctx, *in_ctx;      /* device context, input context */
    oa_u64 out_bus, in_bus;
    struct ring ep[32];           /* by DCI; ring memory from ring_mem */
};

struct xhc {
    struct usb_hc hc;
    volatile oa_u8 *cap, *op, *rt, *db;
    unsigned nports, nslots, ctx_size;
    int ac64;
    oa_u64 *dcbaa;
    oa_u64 dcbaa_bus;
    struct ring cmd;
    volatile struct trb *evt;
    oa_u64 evt_bus;
    unsigned evt_deq;
    oa_u32 evt_cycle;
    oa_u8 *bounce;
    oa_u64 bounce_bus;
    struct slot slots[MAX_SLOTS + 1];
    oa_u32 last_event[4];
    struct xintr *intrs;
    oa_host_event *ev;   /* event-ring MSI-X event, NULL = poll */
};

/* An interrupt IN pipe: one Normal TRB kept queued on the endpoint ring. */
struct xintr {
    struct xhc *x;
    struct slot *s;
    unsigned dci;
    oa_u8 *buf;
    oa_u64 bus, trb;
    oa_u16 len;
    int done;
    oa_u32 cc, residual;
    struct xintr *next;
};

#define MAX_INTR 16
static struct xintr xintrs[MAX_INTR];
static unsigned nxintrs;

static struct xhc xhcs[MAX_CONTROLLERS];
static unsigned nxhcs;

static oa_u32 rd(volatile oa_u8 *b, unsigned off)
{
    return *(volatile oa_u32 *)(b + off);
}

static void wr(volatile oa_u8 *b, unsigned off, oa_u32 v)
{
    *(volatile oa_u32 *)(b + off) = v;
}

static void wr64(volatile oa_u8 *b, unsigned off, oa_u64 v)
{
    wr(b, off, (oa_u32)v);
    wr(b, off + 4u, (oa_u32)(v >> 32));
}

static void *dma(struct xhc *x, oa_u32 size, oa_u32 align, oa_u64 *bus)
{
    void *p = oa_host_dma_alloc(size, align, !x->ac64, bus);
    if (p) {
        memset(p, 0, size);
    }
    return p;
}

static int wait_bits(volatile oa_u8 *b, unsigned off, oa_u32 mask, oa_u32 value, oa_u32 ms)
{
    oa_u64 end = oa_host_time_us() + (oa_u64)ms * 1000u;

    while ((rd(b, off) & mask) != value) {
        if (oa_host_time_us() > end) {
            return -1;
        }
        oa_host_delay_us(50);
    }
    return 0;
}

/* --- rings ------------------------------------------------------------ */

static int ring_init(struct xhc *x, struct ring *r, unsigned ntrbs)
{
    if (r->trb == 0) {
        r->trb = dma(x, ntrbs * 16u, 64, &r->bus);
        if (r->trb == 0) {
            return -1;
        }
    } else {
        memset((void *)r->trb, 0, ntrbs * 16u);
    }
    r->enq = 0;
    r->cycle = 1;
    /* link TRB back to the start, toggling the cycle */
    r->trb[ntrbs - 1u].p0 = (oa_u32)r->bus;
    r->trb[ntrbs - 1u].p1 = (oa_u32)(r->bus >> 32);
    r->trb[ntrbs - 1u].control = (TRB_LINK << 10) | TRB_TC;
    return 0;
}

/* Queue one TRB; returns its bus address. */
static oa_u64 ring_put(struct ring *r, unsigned ntrbs, oa_u32 p0, oa_u32 p1, oa_u32 status,
                       oa_u32 control)
{
    volatile struct trb *t = &r->trb[r->enq];
    oa_u64 addr = r->bus + 16u * r->enq;

    t->p0 = p0;
    t->p1 = p1;
    t->status = status;
    oa_host_mb();
    t->control = (control & ~TRB_CYCLE) | r->cycle;
    if (++r->enq == ntrbs - 1u) { /* hand the link TRB over */
        volatile struct trb *l = &r->trb[r->enq];
        l->control = (l->control & ~TRB_CYCLE) | r->cycle;
        r->enq = 0;
        r->cycle ^= 1u;
    }
    return addr;
}

/* Interrupter 0's MSI-X. An event posted to the ring sets IMAN.IP and raises
 * the interrupt; clear IP (RW1C, keeping IE) and wake whoever waits on the
 * event ring -- the cycle bit is the real condition, this is only a wakeup. */
static volatile int xhci_irq_seen;  /* an xHCI interrupt was delivered + handled */
static int xhci_irq_logged;
static void xhci_irq(void *arg)
{
    struct xhc *x = arg;
    /* MSI-X is a dedicated edge vector, so every interrupt here is ours -- don't
     * gate on IMAN.IP, which QEMU clears when it sends the message (so it already
     * reads 0 by the time we run). Clear it anyway (keeping IE) and wake the
     * waiter; the event ring's cycle bit is the real completion condition. */
    wr(x->rt, IR_IMAN, rd(x->rt, IR_IMAN) | 1u);
    xhci_irq_seen = 1;
    oa_host_event_signal(x->ev);
}

/* Consume the next event if one is on the ring: 0 with *ev filled, or -1 when
 * the ring is empty. Non-blocking -- the caller decides how to wait. */
static int next_event(struct xhc *x, oa_u32 ev[4])
{
    volatile struct trb *t = &x->evt[x->evt_deq];
    oa_u64 deq;

    if ((t->control & TRB_CYCLE) != x->evt_cycle) {
        return -1;
    }
    oa_host_mb();
    ev[0] = t->p0;
    ev[1] = t->p1;
    ev[2] = t->status;
    ev[3] = t->control;
    if (++x->evt_deq == EVT_TRBS) {
        x->evt_deq = 0;
        x->evt_cycle ^= 1u;
    }
    deq = x->evt_bus + 16u * x->evt_deq;
    wr64(x->rt, IR_ERDP, deq | 8u); /* EHB */
    return 0;
}

/* A transfer event for a queued interrupt pipe: note it there. */
static int intr_event(struct xhc *x, const oa_u32 ev[4])
{
    struct xintr *p;
    oa_u64 at = (oa_u64)ev[0] | ((oa_u64)ev[1] << 32);

    if (((ev[3] >> 10) & 0x3Fu) != TRB_EV_TRANSFER) {
        return 0;
    }
    for (p = x->intrs; p; p = p->next) {
        if (!p->done && p->trb == at) {
            p->done = 1;
            p->cc = ev[2] >> 24;
            p->residual = ev[2] & 0xFFFFFFu;
            return 1;
        }
    }
    return 0;
}

/* Wait for the event about the TRB at `trb` (command or transfer). Returns the
 * completion code, 0 on timeout; *ev gets the event. On an interrupt-driven
 * controller it blocks on the interrupter's event, then drains every event the
 * handler made available. Blocking *before* touching the ring matters: the xHC
 * (QEMU's model, and real silicon with interrupt moderation) may raise the
 * interrupt only once the guest stops hammering the ring, so a poll-first loop
 * would drain and clear the pending bit before the interrupt ever fires. A
 * polled controller (no event) just spins. */
static oa_u32 wait_event(struct xhc *x, oa_u32 type, oa_u64 trb, oa_u32 ms, oa_u32 ev[4])
{
    oa_u64 end = oa_host_time_us() + (oa_u64)ms * 1000u, at;

    for (;;) {
        if (x->ev) {
            oa_host_event_wait(x->ev, 1000u); /* block for the completion IRQ */
            if (xhci_irq_seen && !xhci_irq_logged) {
                xhci_irq_logged = 1;
                oa_host_log(OA_LOG_INFO, "xHCI: interrupt-driven completion\n");
            }
        }
        while (next_event(x, ev) == 0) {
            if (intr_event(x, ev)) {
                continue;
            }
            if (((ev[3] >> 10) & 0x3Fu) != type) {
                continue; /* port status changes and the like */
            }
            at = (oa_u64)ev[0] | ((oa_u64)ev[1] << 32);
            if (type == TRB_EV_TRANSFER && trb == 0u) {
                return ev[2] >> 24;
            }
            if (at == trb) {
                return ev[2] >> 24;
            }
            if (type == TRB_EV_TRANSFER) {
                /* an earlier TRB of the same transfer (short packet) */
                x->last_event[0] = ev[0];
                x->last_event[1] = ev[1];
                x->last_event[2] = ev[2];
                x->last_event[3] = ev[3];
                if ((ev[2] >> 24) != CC_SUCCESS && (ev[2] >> 24) != CC_SHORT) {
                    return ev[2] >> 24;
                }
            }
        }
        if (oa_host_time_us() > end) {
            return 0;
        }
    }
}

static oa_u32 command(struct xhc *x, oa_u32 p0, oa_u32 p1, oa_u32 control, oa_u8 *slot_out)
{
    oa_u32 ev[4], cc;
    oa_u64 at = ring_put(&x->cmd, CMD_TRBS, p0, p1, 0, control);

    wr(x->db, 0, 0);
    cc = wait_event(x, TRB_EV_COMMAND, at, 5000, ev);
    if (slot_out) {
        *slot_out = (oa_u8)(ev[3] >> 24);
    }
    return cc;
}

/* --- contexts ---------------------------------------------------------- */

static oa_u32 *in_ctx(struct xhc *x, struct slot *s, unsigned entry)
{
    /* entry 0: input control, 1: slot, 1 + dci: endpoint */
    return (oa_u32 *)(s->in_ctx + entry * x->ctx_size);
}

static void slot_ctx(struct xhc *x, struct slot *s, unsigned entries)
{
    struct usb_dev *d = s->dev;
    oa_u32 *c = in_ctx(x, s, 1);

    memset(c, 0, x->ctx_size);
    c[0] = (d->route & 0xFFFFFu) | ((oa_u32)d->speed << 20) | ((oa_u32)entries << 27);
    if (d->is_hub) {
        c[0] |= 1ul << 26;
    }
    c[1] = (oa_u32)d->root_port << 16;
    if (d->is_hub) {
        c[1] |= (oa_u32)d->hub_ports << 24;
    }
    if (d->tt_hub && d->tt_hub->hcpriv) {
        c[2] = ((struct slot *)d->tt_hub->hcpriv)->id | ((oa_u32)d->tt_port << 8);
    }
}

static int ep_ctx(struct xhc *x, struct slot *s, unsigned dci, oa_u32 type, oa_u32 mps,
                  oa_u32 interval)
{
    oa_u32 *c = in_ctx(x, s, 1u + dci);
    struct ring *r = &s->ep[dci];

    if (ring_init(x, r, RING_TRBS) < 0) {
        return -1;
    }
    memset(c, 0, x->ctx_size);
    c[0] = interval << 16;
    c[1] = (3ul << 1) | (type << 3) | (mps << 16); /* 3 retries */
    c[2] = (oa_u32)r->bus | 1u;                  /* dequeue, cycle 1 */
    c[3] = (oa_u32)(r->bus >> 32);
    c[4] = type == 4u ? 8u : (type == 3u || type == 7u) ? (mps | (mps << 16)) : 1024u;
    return 0;
}

static struct slot *slot_of(struct usb_dev *d)
{
    return d->hcpriv;
}

/* --- root ports -------------------------------------------------------- */

static unsigned x_ports(struct usb_hc *hc)
{
    return ((struct xhc *)hc->priv)->nports;
}

static int x_port_reset(struct usb_hc *hc, unsigned port, oa_u8 *speed)
{
    struct xhc *x = hc->priv;
    unsigned off = OP_PORTSC + 0x10u * (port - 1u);
    oa_u32 v = rd(x->op, off);
    oa_u32 pls;

    if (!(v & PORT_CCS)) {
        return -1;
    }
    if (!(v & PORT_PED)) {
        pls = (v >> 5) & 0xFu;
        if (((v >> 10) & 0xFu) >= USB_SPEED_SUPER || pls == 5u /* RxDetect */ || pls == 6u) {
            wr(x->op, off, (v & PORT_KEEP) | PORT_WPR); /* warm reset */
        } else {
            wr(x->op, off, (v & PORT_KEEP) | PORT_PR);
        }
        if (wait_bits(x->op, off, PORT_PRC | PORT_WRC, PORT_PRC, 500) < 0 &&
            !(rd(x->op, off) & PORT_WRC)) {
            return -1;
        }
        oa_host_delay_us(20000);
    }
    v = rd(x->op, off);
    wr(x->op, off, (v & PORT_KEEP) | (v & PORT_CHANGES)); /* clear the changes */
    if ((v & (PORT_CCS | PORT_PED)) != (PORT_CCS | PORT_PED)) {
        return -1;
    }
    *speed = (oa_u8)((v >> 10) & 0xFu);
    if (*speed == 0u || *speed > USB_SPEED_SUPER) {
        *speed = USB_SPEED_SUPER; /* SuperSpeedPlus as super */
    }
    return 0;
}

/* --- device slots -------------------------------------------------------- */

static int x_address(struct usb_hc *hc, struct usb_dev *d)
{
    struct xhc *x = hc->priv;
    struct slot *s;
    oa_u8 id = 0;
    oa_u32 *icc;

    if (command(x, 0, 0, TRB_ENABLE_SLOT << 10, &id) != CC_SUCCESS || id == 0u || id > x->nslots ||
        id > MAX_SLOTS) {
        return -1;
    }
    s = &x->slots[id];
    if (s->out_ctx == 0) {
        s->out_ctx = dma(x, PAGE, 64, &s->out_bus);
        s->in_ctx = dma(x, PAGE, 64, &s->in_bus);
        if (s->out_ctx == 0 || s->in_ctx == 0) {
            return -1;
        }
    } else {
        memset(s->out_ctx, 0, PAGE);
    }
    s->id = id;
    s->dev = d;
    d->hcpriv = s;
    x->dcbaa[id] = s->out_bus;

    memset(s->in_ctx, 0, PAGE);
    icc = in_ctx(x, s, 0);
    icc[1] = 3u; /* add slot and EP0 */
    slot_ctx(x, s, 1);
    if (ep_ctx(x, s, 1, 4u, d->out[0].mps, 0) < 0) {
        return -1;
    }
    oa_host_mb();
    if (command(x, (oa_u32)s->in_bus, (oa_u32)(s->in_bus >> 32),
                (TRB_ADDRESS << 10) | ((oa_u32)id << 24), 0) != CC_SUCCESS) {
        return -1;
    }
    d->addr = (oa_u8)(((volatile oa_u32 *)s->out_ctx)[3] & 0xFFu);
    oa_host_delay_us(2000);
    return 0;
}

static int x_update_ep0(struct usb_hc *hc, struct usb_dev *d)
{
    struct xhc *x = hc->priv;
    struct slot *s = slot_of(d);
    oa_u32 *icc, *c;

    memset(s->in_ctx, 0, PAGE);
    icc = in_ctx(x, s, 0);
    icc[1] = 2u; /* EP0 */
    c = in_ctx(x, s, 2);
    c[1] = (3ul << 1) | (4ul << 3) | ((oa_u32)d->out[0].mps << 16);
    oa_host_mb();
    return command(x, (oa_u32)s->in_bus, (oa_u32)(s->in_bus >> 32),
                   (TRB_EVALUATE << 10) | ((oa_u32)s->id << 24), 0) == CC_SUCCESS ? 0 : -1;
}

/* bInterval to the xHCI interval (2^n x 125 us) */
static oa_u32 ep_interval(const struct usb_dev *d, const struct usb_ep *ep)
{
    unsigned i, frames;

    if (d->speed == USB_SPEED_HIGH || d->speed == USB_SPEED_SUPER) {
        return ep->interval ? (oa_u32)(ep->interval - 1u) : 0u;
    }
    frames = ep->interval ? ep->interval : 1u; /* ms */
    for (i = 0; (8u << i) <= frames * 8u && i < 15u; i++) {
        if ((1u << i) >= frames * 8u) {
            break;
        }
    }
    return i + 3u > 10u ? 10u : i + 3u;
}

static int x_configure(struct usb_hc *hc, struct usb_dev *d)
{
    struct xhc *x = hc->priv;
    struct slot *s = slot_of(d);
    oa_u32 *icc, add = 1u, type;
    unsigned n, dci, maxdci = 1;
    struct usb_ep *ep;

    memset(s->in_ctx, 0, PAGE);
    for (n = 1; n < USB_MAX_EP; n++) {
        ep = &d->out[n];
        if (ep->mps && ep->type != USB_EP_ISOCH) {
            dci = 2u * n;
            type = ep->type == USB_EP_BULK ? 2u : 3u;
            if (ep_ctx(x, s, dci, type, ep->mps, ep->type == USB_EP_INTR ? ep_interval(d, ep) : 0) < 0) {
                return -1;
            }
            add |= 1ul << dci;
            if (dci > maxdci) {
                maxdci = dci;
            }
        }
        ep = &d->in[n];
        if (ep->mps && ep->type != USB_EP_ISOCH) {
            dci = 2u * n + 1u;
            type = ep->type == USB_EP_BULK ? 6u : 7u;
            if (ep_ctx(x, s, dci, type, ep->mps, ep->type == USB_EP_INTR ? ep_interval(d, ep) : 0) < 0) {
                return -1;
            }
            add |= 1ul << dci;
            if (dci > maxdci) {
                maxdci = dci;
            }
        }
    }
    icc = in_ctx(x, s, 0);
    icc[1] = add;
    slot_ctx(x, s, maxdci);
    oa_host_mb();
    return command(x, (oa_u32)s->in_bus, (oa_u32)(s->in_bus >> 32),
                   (TRB_CONFIGURE << 10) | ((oa_u32)s->id << 24), 0) == CC_SUCCESS ? 0 : -1;
}

static void x_release(struct usb_hc *hc, struct usb_dev *d)
{
    struct xhc *x = hc->priv;
    struct slot *s = slot_of(d);

    if (s) {
        command(x, 0, 0, (TRB_DISABLE_SLOT << 10) | ((oa_u32)s->id << 24), 0);
        x->dcbaa[s->id] = 0;
        s->dev = 0;
        d->hcpriv = 0;
    }
}

/* Point an endpoint at a fresh ring. Set TR Dequeue needs it stopped:
 * Reset Endpoint stops a halted one (a stall, an error), Stop Endpoint a
 * running one (a timeout); each fails harmlessly in the other state. */
static void reset_ep(struct xhc *x, struct slot *s, unsigned dci)
{
    struct ring *r = &s->ep[dci];

    command(x, 0, 0, (TRB_RESET_EP << 10) | ((oa_u32)dci << 16) | ((oa_u32)s->id << 24), 0);
    command(x, 0, 0, (TRB_STOP_EP << 10) | ((oa_u32)dci << 16) | ((oa_u32)s->id << 24), 0);
    ring_init(x, r, RING_TRBS);
    oa_host_mb();
    command(x, (oa_u32)r->bus | 1u, (oa_u32)(r->bus >> 32),
            (TRB_SET_DEQ << 10) | ((oa_u32)dci << 16) | ((oa_u32)s->id << 24), 0);
}

static int result(oa_u32 cc)
{
    if (cc == CC_SUCCESS || cc == CC_SHORT) {
        return USB_OK;
    }
    if (cc == 0u) {
        return USB_TIMEOUT;
    }
    return cc == CC_STALL ? USB_STALL : USB_ERROR;
}

static int x_control(struct usb_hc *hc, struct usb_dev *d, const oa_u8 setup[8], void *data,
                     oa_u16 len, oa_u16 *actual, oa_u32 timeout_ms)
{
    struct xhc *x = hc->priv;
    struct slot *s = slot_of(d);
    struct ring *r = &s->ep[1];
    oa_u32 ev[4], cc, trt, p0, p1, residual = 0;
    oa_u64 at;
    int in = (setup[0] & 0x80u) != 0;

    *actual = 0; /* len (16 bits) always fits the bounce buffer */
    if (!in && len) {
        memcpy(x->bounce, data, len);
    }
    p0 = (oa_u32)setup[0] | ((oa_u32)setup[1] << 8) | ((oa_u32)setup[2] << 16) |
         ((oa_u32)setup[3] << 24);
    p1 = (oa_u32)setup[4] | ((oa_u32)setup[5] << 8) | ((oa_u32)setup[6] << 16) |
         ((oa_u32)setup[7] << 24);
    trt = len == 0u ? 0u : in ? 3u : 2u;
    x->last_event[0] = x->last_event[1] = x->last_event[2] = x->last_event[3] = 0;
    ring_put(r, RING_TRBS, p0, p1, 8u, (TRB_SETUP << 10) | TRB_IDT | (trt << 16));
    if (len) {
        ring_put(r, RING_TRBS, (oa_u32)x->bounce_bus, (oa_u32)(x->bounce_bus >> 32), len,
                 (TRB_DATA << 10) | TRB_ISP | (in ? TRB_DIR_IN : 0u));
    }
    at = ring_put(r, RING_TRBS, 0, 0, 0,
                  (TRB_STATUS << 10) | TRB_IOC | ((len && in) ? 0u : TRB_DIR_IN));
    oa_host_mb();
    wr(x->db, 4u * s->id, 1);
    cc = wait_event(x, TRB_EV_TRANSFER, at, timeout_ms ? timeout_ms : 5000u, ev);
    if (cc != CC_SUCCESS && cc != CC_SHORT) {
        if (cc == CC_STALL || cc == 0u || cc != CC_SUCCESS) {
            reset_ep(x, s, 1);
        }
        return result(cc);
    }
    if (len) {
        if (((x->last_event[2] >> 24) == CC_SHORT)) {
            residual = x->last_event[2] & 0xFFFFFFu;
        } else if (cc == CC_SHORT) {
            residual = ev[2] & 0xFFFFFFu;
        }
        *actual = (oa_u16)(residual < len ? len - residual : 0u);
        if (in && *actual) {
            memcpy(data, x->bounce, *actual);
        }
    }
    return USB_OK;
}

static int x_transfer(struct usb_hc *hc, struct usb_dev *d, oa_u8 ep, void *data, oa_u32 len,
                      oa_u32 *actual, oa_u32 timeout_ms)
{
    struct xhc *x = hc->priv;
    struct slot *s = slot_of(d);
    unsigned dci = 2u * (ep & 15u) + ((ep & 0x80u) ? 1u : 0u);
    struct ring *r = &s->ep[dci];
    oa_u32 ev[4], cc, n;
    oa_u64 at;
    int in = (ep & 0x80u) != 0;

    *actual = 0;
    if (len > BOUNCE_SIZE || r->trb == 0) {
        return USB_ERROR;
    }
    if (!in && len) {
        memcpy(x->bounce, data, len);
    }
    at = ring_put(r, RING_TRBS, (oa_u32)x->bounce_bus, (oa_u32)(x->bounce_bus >> 32), len,
                  (TRB_NORMAL << 10) | TRB_ISP | TRB_IOC);
    oa_host_mb();
    wr(x->db, 4u * s->id, dci);
    cc = wait_event(x, TRB_EV_TRANSFER, at, timeout_ms ? timeout_ms : 15000u, ev);
    if (cc != CC_SUCCESS && cc != CC_SHORT) {
        if (cc == 0u) {
            reset_ep(x, s, dci); /* a stalled endpoint is reset by clear_halt */
        }
        return result(cc);
    }
    n = ev[2] & 0xFFFFFFu; /* residual */
    *actual = n < len ? len - n : 0u;
    if (in && *actual) {
        memcpy(data, x->bounce, *actual);
    }
    return USB_OK;
}

static void x_clear_halt(struct usb_hc *hc, struct usb_dev *d, oa_u8 ep)
{
    struct xhc *x = hc->priv;
    struct slot *s = slot_of(d);
    unsigned dci = (ep & 15u) == 0u ? 1u : 2u * (ep & 15u) + ((ep & 0x80u) ? 1u : 0u);

    if (s && s->ep[dci].trb) {
        reset_ep(x, s, dci);
    }
}

static void intr_arm(struct xintr *p)
{
    p->done = 0;
    p->trb = ring_put(&p->s->ep[p->dci], RING_TRBS, (oa_u32)p->bus, (oa_u32)(p->bus >> 32), p->len,
                      (TRB_NORMAL << 10) | TRB_ISP | TRB_IOC);
    oa_host_mb();
    wr(p->x->db, 4u * p->s->id, p->dci);
}

static int x_intr_open(struct usb_hc *hc, struct usb_intr *pipe)
{
    struct xhc *x = hc->priv;
    struct slot *s = slot_of(pipe->dev);
    struct xintr *p;
    unsigned dci = 2u * (pipe->ep & 15u) + 1u;

    if (nxintrs == MAX_INTR || s == 0 || s->ep[dci].trb == 0) {
        return -1;
    }
    p = &xintrs[nxintrs];
    p->buf = oa_host_dma_alloc(1024, 64, !x->ac64, &p->bus);
    if (p->buf == 0) {
        return -1;
    }
    nxintrs++;
    p->x = x;
    p->s = s;
    p->dci = dci;
    p->len = pipe->len;
    p->next = x->intrs;
    x->intrs = p;
    pipe->hcpriv = p;
    intr_arm(p);
    return 0;
}

static int x_intr_poll(struct usb_hc *hc, struct usb_intr *pipe, void *buf, oa_u16 *actual)
{
    struct xhc *x = hc->priv;
    struct xintr *p = pipe->hcpriv;
    oa_u32 ev[4];

    while (!p->done && next_event(x, ev) == 0) {
        intr_event(x, ev); /* nobody else is waiting now */
    }
    if (!p->done) {
        return 0;
    }
    if (p->cc != CC_SUCCESS && p->cc != CC_SHORT) {
        oa_u32 cc = p->cc;
        reset_ep(x, p->s, p->dci);
        intr_arm(p);
        return -result(cc);
    }
    *actual = (oa_u16)(p->residual < p->len ? p->len - p->residual : 0u);
    memcpy(buf, p->buf, *actual);
    intr_arm(p);
    return 1;
}

static const struct usb_hc_ops xhci_ops = {
    x_ports, x_port_reset, x_address, x_update_ep0, x_configure,
    x_control, x_transfer, x_clear_halt, x_release, x_intr_open, x_intr_poll
};

/* --- bring-up ------------------------------------------------------------ */

static void bios_handoff(struct xhc *x)
{
    oa_u32 off = (rd(x->cap, CAP_HCC1) >> 16) << 2, v;
    unsigned guard = 0;

    while (off != 0u && guard++ < 64u) {
        v = rd(x->cap, off);
        if ((v & 0xFFu) == 1u) { /* USB legacy support */
            if (v & (1ul << 16)) {
                wr(x->cap, off, v | (1ul << 24));
                if (wait_bits(x->cap, off, 1ul << 16, 0, 1000) < 0) {
                    wr(x->cap, off, (rd(x->cap, off) & ~(1ul << 16)) | (1ul << 24));
                }
            }
            /* no SMIs, clear their status */
            wr(x->cap, off + 4u, (rd(x->cap, off + 4u) & 0x000E1FEEul) | 0xE0000000ul);
            return;
        }
        if (((v >> 8) & 0xFFu) == 0u) {
            return;
        }
        off += ((v >> 8) & 0xFFu) << 2;
    }
}

static int scratchpads(struct xhc *x)
{
    oa_u32 hcs2 = rd(x->cap, CAP_HCS2);
    unsigned n = ((hcs2 >> 27) & 0x1Fu) | (((hcs2 >> 21) & 0x1Fu) << 5), i;
    oa_u64 *arr, arr_bus, bus;

    if (n == 0u) {
        return 0;
    }
    arr = dma(x, PAGE, 64, &arr_bus);
    if (arr == 0) {
        return -1;
    }
    for (i = 0; i < n && i < PAGE / 8u; i++) {
        if (dma(x, PAGE, PAGE, &bus) == 0) {
            return -1;
        }
        arr[i] = bus;
    }
    x->dcbaa[0] = arr_bus;
    return 0;
}

static void hex(char *out, oa_u32 v, unsigned digits)
{
    while (digits--) {
        out[digits] = "0123456789abcdef"[v & 15u];
        v >>= 4;
    }
}

int usb_add_xhci(oa_u32 pci)
{
    struct xhc *x;
    oa_u32 bar_lo, bar_hi = 0, cmd, hcs1, *erst;
    oa_u64 bar, erst_bus;
    unsigned i;

    if (nxhcs == MAX_CONTROLLERS ||
        (oa_host_pci_read32(pci, 0x08) >> 8) != ((oa_u32)USB_PCI_CLASS << 8 | USB_PCI_XHCI)) {
        return -1;
    }
    bar_lo = oa_host_pci_read32(pci, 0x10);
    if ((bar_lo & 0x6u) == 0x4u) {
        bar_hi = oa_host_pci_read32(pci, 0x14);
    }
    bar = ((oa_u64)bar_hi << 32) | (bar_lo & 0xFFFFFFF0ul);
    if (bar == 0u || (bar_lo & 1u)) {
        return -1;
    }
    cmd = oa_host_pci_read32(pci, 0x04);
    oa_host_pci_write32(pci, 0x04, (cmd & 0xFFFFu) | 0x0006u);

    x = &xhcs[nxhcs];
    memset(x, 0, sizeof(*x));
    x->cap = oa_host_map_mmio(bar, 0x10000);
    if (x->cap == 0) {
        return -1;
    }
    x->op = x->cap + (rd(x->cap, CAP_LENGTH) & 0xFFu);
    x->rt = x->cap + (rd(x->cap, CAP_RTSOFF) & ~0x1Ful);
    x->db = x->cap + (rd(x->cap, CAP_DBOFF) & ~0x3ul);
    hcs1 = rd(x->cap, CAP_HCS1);
    x->nslots = hcs1 & 0xFFu;
    if (x->nslots > MAX_SLOTS) {
        x->nslots = MAX_SLOTS;
    }
    x->nports = hcs1 >> 24;
    x->ac64 = rd(x->cap, CAP_HCC1) & 1u;
    x->ctx_size = (rd(x->cap, CAP_HCC1) & 4u) ? 64u : 32u;
    if (!(rd(x->op, OP_PAGESIZE) & 1u)) {
        return -1; /* no 4 KiB pages */
    }

    bios_handoff(x);
    /* stop, reset */
    wr(x->op, OP_USBCMD, rd(x->op, OP_USBCMD) & ~CMD_RS);
    wait_bits(x->op, OP_USBSTS, STS_HCH, STS_HCH, 100);
    wr(x->op, OP_USBCMD, CMD_HCRST);
    if (wait_bits(x->op, OP_USBCMD, CMD_HCRST, 0, 1000) < 0 ||
        wait_bits(x->op, OP_USBSTS, STS_CNR, 0, 1000) < 0) {
        oa_host_log(OA_LOG_WARN, "xHCI: controller does not come out of reset\n");
        return -1;
    }

    x->dcbaa = dma(x, PAGE, 64, &x->dcbaa_bus);
    x->bounce = dma(x, BOUNCE_SIZE, BOUNCE_SIZE, &x->bounce_bus); /* no 64 KiB crossing */
    x->evt = dma(x, EVT_TRBS * 16u, 64, &x->evt_bus);
    erst = dma(x, 64, 64, &erst_bus);
    if (x->dcbaa == 0 || x->bounce == 0 || x->evt == 0 || erst == 0 ||
        ring_init(x, &x->cmd, CMD_TRBS) < 0 || scratchpads(x) < 0) {
        oa_host_log(OA_LOG_ERROR, "xHCI: out of DMA memory\n");
        return -1;
    }
    wr(x->op, OP_CONFIG, x->nslots);
    wr64(x->op, OP_DCBAAP, x->dcbaa_bus);
    wr64(x->op, OP_CRCR, x->cmd.bus | 1u);
    erst[0] = (oa_u32)x->evt_bus;
    erst[1] = (oa_u32)(x->evt_bus >> 32);
    erst[2] = EVT_TRBS;
    x->evt_cycle = 1;
    wr(x->rt, IR_IMAN, 1);      /* clear pending, no interrupts yet */
    wr(x->rt, IR_ERSTSZ, 1);
    wr64(x->rt, IR_ERDP, x->evt_bus);
    wr64(x->rt, IR_ERSTBA, erst_bus);
    /* Interrupt-driven event ring: bind one MSI-X vector to interrupter 0 and
     * enable it (IMAN.IE). On any failure ev stays NULL and the ring is polled,
     * with the interrupter and the host system interrupt (CMD_INTE) left off,
     * so a controller without working MSI-X never regresses. */
    x->ev = oa_host_event_create();
    if (x->ev && oa_host_pci_msix(pci, xhci_irq, x) >= 0) {
        wr(x->rt, IR_IMOD, 0);    /* no moderation: deliver each event promptly */
        wr(x->rt, IR_IMAN, 0x3u); /* clear IP (bit0 RW1C) + enable IE (bit1) */
    } else if (x->ev) {
        oa_host_event_destroy(x->ev);
        x->ev = 0;
    }
    oa_host_mb();
    wr(x->op, OP_USBCMD, x->ev ? (CMD_RS | CMD_INTE) : CMD_RS);
    if (wait_bits(x->op, OP_USBSTS, STS_HCH, 0, 100) < 0) {
        return -1;
    }
    /* power the ports, give the links time to come up */
    for (i = 0; i < x->nports; i++) {
        oa_u32 v = rd(x->op, OP_PORTSC + 0x10u * i);
        if (!(v & PORT_PP)) {
            wr(x->op, OP_PORTSC + 0x10u * i, (v & PORT_KEEP) | PORT_PP);
        }
    }
    oa_host_delay_us(100000);

    memcpy(x->hc.name, "xHCI 0000:00:00.0", 18);
    hex(x->hc.name + 5, pci >> 16, 4);
    hex(x->hc.name + 10, (pci >> 8) & 0xFFu, 2);
    hex(x->hc.name + 13, (pci >> 3) & 0x1Fu, 2);
    x->hc.name[16] = (char)('0' + (pci & 7u));
    x->hc.ops = &xhci_ops;
    x->hc.priv = x;
    x->hc.max_xfer = BOUNCE_SIZE;
    nxhcs++;
    usb_hc_register(&x->hc);
    return 0;
}
