/* SPDX-License-Identifier: GPL-2.0-or-later */
/* UHCI host controller for eusbcore, the native USB stack (polled, port
 * I/O).
 *
 * As EUSBASPI.SYS's uhci_init: legacy support off in PCI configuration
 * space (no SMIs, no keyboard emulation), global and host controller
 * reset, a frame list whose 1024 entries all point to one queue head. A
 * transfer is a chain of TDs of one packet each, hung into that queue head
 * depth first and polled; the data toggle is kept per endpoint. A
 * transfer is at most one page. */

#include <string.h>

#include "eusbcore/usb.h"
#include "openaspi/native/host.h"

#define MAX_CONTROLLERS 8
#define PAGE   4096u
#define NTD    80u    /* one page in 64-byte packets, setup and status */

#define R_CMD    0x00u
#define R_STS    0x02u
#define R_INTR   0x04u
#define R_FRNUM  0x06u
#define R_FLBASE 0x08u
#define R_SOF    0x0Cu
#define R_PORT   0x10u

#define CMD_RS     0x01u
#define CMD_HCRST  0x02u
#define CMD_GRESET 0x04u
#define CMD_CF     0x40u
#define CMD_MAXP   0x80u
#define STS_HALTED 0x20u

#define P_CCS  0x0001u
#define P_CSC  0x0002u
#define P_PE   0x0004u
#define P_PEC  0x0008u
#define P_LSDA 0x0100u
#define P_PR   0x0200u
#define P_WC   (P_CSC | P_PEC)

#define TD_ACTIVE  (1ul << 23)
#define TD_STALLED (1ul << 22)
#define TD_ERRORS  (0x7Eul << 16) /* stalled, buffer, babble, NAK, CRC, bitstuff */
#define TD_LS      (1ul << 26)
#define TD_SPD     (1ul << 29)
#define TD_IOC     (1ul << 24)    /* interrupt on completion */
#define STS_INT    0x0001u        /* USBSTS: a TD with IOC retired */
#define STS_ERR    0x0002u        /* USBSTS: USB error interrupt */
#define INTR_TMO   0x0001u        /* USBINTR: timeout/CRC enable */
#define INTR_IOC   0x0004u        /* USBINTR: interrupt-on-completion enable */
#define PID_IN    0x69u
#define PID_OUT   0xE1u
#define PID_SETUP 0x2Du

struct td {
    oa_u32 link, cs, token, buf;
};

struct uhc {
    struct usb_hc hc;
    oa_u16 io;
    volatile oa_u32 *qh;    /* head link, element link */
    oa_u64 qh_bus;
    volatile struct td *td;
    oa_u64 td_bus;
    oa_u8 *bounce;
    oa_u64 bounce_bus;
    oa_u32 *frames;
    oa_host_event *ev;      /* completion INTx event, NULL = polled */
};

/* An interrupt IN pipe: a queue head in every frame, in front of the
 * control/bulk queue head, with one TD re-armed after each report. */
struct uintr {
    volatile oa_u32 *qh;
    oa_u64 qh_bus;
    volatile struct td *td;
    oa_u64 td_bus;
    oa_u8 *buf;
    oa_u64 buf_bus;
    oa_u16 len;
    struct usb_ep *ep;
    int low;
    oa_u32 token;     /* without the toggle */
};

#define MAX_INTR 16
static struct uintr uintrs[MAX_INTR];
static unsigned nuintrs;

static struct uhc uhcs[MAX_CONTROLLERS];
static unsigned nuhcs;

static oa_u16 inw(struct uhc *u, unsigned r)
{
    return oa_host_inw((oa_u16)(u->io + r));
}

static void outw(struct uhc *u, unsigned r, oa_u16 v)
{
    oa_host_outw((oa_u16)(u->io + r), v);
}

static unsigned u_ports(struct usb_hc *hc)
{
    (void)hc;
    return 2;
}

static int u_port_reset(struct usb_hc *hc, unsigned port, oa_u8 *speed)
{
    struct uhc *u = hc->priv;
    unsigned r = R_PORT + 2u * (port - 1u), i;
    oa_u16 v = inw(u, r);

    if (!(v & P_CCS)) {
        return -1;
    }
    outw(u, r, (oa_u16)((v & ~P_WC) | P_PR));
    oa_host_delay_us(50000);
    outw(u, r, (oa_u16)(inw(u, r) & ~(P_WC | P_PR)));
    oa_host_delay_us(100);
    for (i = 0; i < 10u; i++) {
        v = inw(u, r);
        if (!(v & P_CCS)) {
            return -1;
        }
        if (v & P_WC) {
            outw(u, r, (oa_u16)(v & ~P_PR)); /* clear the changes */
            continue;
        }
        if (v & P_PE) {
            break;
        }
        outw(u, r, (oa_u16)(v | P_PE));
        oa_host_delay_us(10000);
    }
    v = inw(u, r);
    if (!(v & P_PE)) {
        return -1;
    }
    oa_host_delay_us(10000);
    *speed = (v & P_LSDA) ? USB_SPEED_LOW : USB_SPEED_FULL;
    return 0;
}

static int u_address(struct usb_hc *hc, struct usb_dev *dev)
{
    return usb_set_address(hc, dev);
}

/* TD i for `len` bytes (<= max packet) at bus address `bus`. */
static void fill(struct uhc *u, const struct usb_dev *d, unsigned i, oa_u8 pid, oa_u8 ep,
                 oa_u32 toggle, oa_u64 bus, oa_u32 len)
{
    volatile struct td *t = &u->td[i];

    t->link = (oa_u32)(u->td_bus + 16u * (i + 1u)) | 4u; /* depth first */
    t->cs = TD_ACTIVE | TD_SPD | (3ul << 27) | (d->speed == USB_SPEED_LOW ? TD_LS : 0u);
    t->token = pid | ((oa_u32)d->addr << 8) | ((oa_u32)(ep & 15u) << 15) | (toggle << 19) |
               ((len ? len - 1u : 0x7FFu) << 21);
    t->buf = (oa_u32)bus;
}

static volatile int uhci_irq_seen;  /* a UHCI interrupt was delivered + handled */
static int uhci_irq_logged;

/* Log once that a completion interrupt was delivered. Checked while waiting AND
 * when a transfer is done: on a fast host every transfer can already be complete
 * at the first look, so the wait (and a check only there) is never reached. */
static void uhci_irq_proof(void)
{
    if (uhci_irq_seen && !uhci_irq_logged) {
        uhci_irq_logged = 1;
        oa_host_log(OA_LOG_INFO, "UHCI: interrupt-driven completion\n");
    }
}

/* INTx handler: shared and level-triggered. Acknowledge our USBSTS bits (RW1C,
 * which de-asserts the level line) and wake the waiter; ignore an interrupt
 * with none of our bits set (another device on the shared line). */
static void uhci_irq(void *arg)
{
    struct uhc *u = arg;
    oa_u16 sts = inw(u, R_STS) & (STS_INT | STS_ERR);

    if (!sts) {
        return;
    }
    outw(u, R_STS, sts);
    uhci_irq_seen = 1;
    if (u->ev) {
        oa_host_event_signal(u->ev);
    }
}

/* Run td[0..n-1]; *done gets the number of TDs completed, *bytes their
 * data. Stops at a short packet. */
static int run(struct uhc *u, unsigned n, oa_u32 timeout_ms, unsigned *done, oa_u32 *bytes)
{
    oa_u64 end = oa_host_time_us() + (oa_u64)timeout_ms * 1000u;
    oa_u32 cs, maxlen, got;
    unsigned i;
    int r = USB_OK;

    u->td[n - 1u].link = 1u;
    /* Interrupt-driven: the last TD raises USBINT on completion so the waiter
     * can block instead of spinning. Harmless when polled (nothing reads it). */
    if (u->ev) {
        u->td[n - 1u].cs |= TD_IOC;
    }
    oa_host_mb();
    u->qh[1] = (oa_u32)u->td_bus;
    *done = 0;
    *bytes = 0;
    for (;;) {
        for (i = *done; i < n; i++) {
            cs = u->td[i].cs;
            if (cs & TD_ACTIVE) {
                break;
            }
            if (cs & TD_ERRORS) {
                r = (cs & TD_STALLED) ? USB_STALL : USB_ERROR;
                goto stop;
            }
            got = (cs + 1u) & 0x7FFu;
            maxlen = ((u->td[i].token >> 21) + 1u) & 0x7FFu;
            *bytes += got;
            *done = i + 1u;
            if (got < maxlen) {
                goto stop; /* short packet */
            }
        }
        if (*done == n) {
            break;
        }
        /* Block for the completion IRQ (poll-first: the TD status is in memory,
         * the interrupt just wakes us). A polled controller spins. */
        if (u->ev) {
            oa_host_event_wait(u->ev, 10u);
            uhci_irq_proof();
        }
        if (oa_host_time_us() > end) {
            r = USB_TIMEOUT;
            goto stop;
        }
    }
stop:
    uhci_irq_proof();
    u->qh[1] = 1u; /* out of the schedule */
    oa_host_mb();
    oa_host_delay_us(1000); /* one frame: the controller lets go */
    return r;
}

static int u_control(struct usb_hc *hc, struct usb_dev *d, const oa_u8 setup[8], void *data,
                     oa_u16 len, oa_u16 *actual, oa_u32 timeout_ms)
{
    struct uhc *u = hc->priv;
    int in = (setup[0] & 0x80u) != 0, r;
    unsigned n = 0, done;
    oa_u32 mps = d->out[0].mps ? d->out[0].mps : 8u, off, chunk, bytes, toggle = 1, data_bytes;

    *actual = 0;
    if (len > PAGE - 64u || (len + mps - 1u) / mps + 2u > NTD) {
        return USB_ERROR;
    }
    memcpy(u->bounce, setup, 8);
    if (!in && len) {
        memcpy(u->bounce + 64, data, len);
    }
    fill(u, d, n++, PID_SETUP, 0, 0, u->bounce_bus, 8);
    for (off = 0; off < len; off += chunk) {
        chunk = len - off < mps ? len - off : mps;
        fill(u, d, n++, in ? PID_IN : PID_OUT, 0, toggle, u->bounce_bus + 64u + off, chunk);
        toggle ^= 1u;
    }
    fill(u, d, n++, (len && in) ? PID_OUT : PID_IN, 0, 1, 0, 0);
    r = run(u, n, timeout_ms ? timeout_ms : 5000u, &done, &bytes);
    if (r != USB_OK) {
        return r;
    }
    data_bytes = bytes - 8u; /* without the setup packet */
    if (done < n) {
        /* a short data packet stopped the chain: the status stage alone */
        unsigned d2;
        oa_u32 b2;
        oa_u32 link_token = u->td[n - 1u].token;
        fill(u, d, 0, (oa_u8)(link_token & 0xFFu), 0, 1, 0, 0);
        r = run(u, 1, timeout_ms ? timeout_ms : 5000u, &d2, &b2);
        if (r != USB_OK) {
            return r;
        }
    } else {
        data_bytes = bytes - 8u; /* the status packet carries nothing */
    }
    if (len) {
        *actual = (oa_u16)(data_bytes > len ? len : data_bytes);
        if (in && *actual) {
            memcpy(data, u->bounce + 64, *actual);
        }
    }
    return USB_OK;
}

static int u_transfer(struct usb_hc *hc, struct usb_dev *d, oa_u8 ep, void *data, oa_u32 len,
                      oa_u32 *actual, oa_u32 timeout_ms)
{
    struct uhc *u = hc->priv;
    struct usb_ep *uep = (ep & 0x80u) ? &d->in[ep & 15u] : &d->out[ep & 15u];
    int in = (ep & 0x80u) != 0, r;
    unsigned n = 0, done, i;
    oa_u32 mps = uep->mps ? uep->mps : 64u, off = 0, chunk, bytes;

    *actual = 0;
    if (len > PAGE || (len + mps - 1u) / mps > NTD) {
        return USB_ERROR;
    }
    if (!in && len) {
        memcpy(u->bounce, data, len);
    }
    do {
        chunk = len - off < mps ? len - off : mps;
        fill(u, d, n, in ? PID_IN : PID_OUT, ep, (uep->toggle + n) & 1u, u->bounce_bus + off, chunk);
        n++;
        off += chunk;
    } while (off < len);
    r = run(u, n, timeout_ms ? timeout_ms : 15000u, &done, &bytes);
    for (i = 0; i < done; i++) {
        uep->toggle ^= 1u;
    }
    *actual = bytes;
    if (in && bytes) {
        memcpy(data, u->bounce, bytes);
    }
    return r;
}

static void intr_arm(struct uintr *p)
{
    p->td->link = 1u;
    p->td->buf = (oa_u32)p->buf_bus;
    p->td->token = p->token | ((oa_u32)p->ep->toggle << 19);
    oa_host_mb();
    p->td->cs = TD_ACTIVE | TD_SPD | (3ul << 27) | (p->low ? TD_LS : 0u);
    oa_host_mb();
    p->qh[1] = (oa_u32)p->td_bus;
    oa_host_mb();
}

static int u_intr_open(struct usb_hc *hc, struct usb_intr *pipe)
{
    struct uhc *u = hc->priv;
    struct usb_dev *d = pipe->dev;
    struct uintr *p;
    unsigned i;

    if (nuintrs == MAX_INTR || u->frames == 0) {
        return -1;
    }
    p = &uintrs[nuintrs];
    p->qh = oa_host_dma_alloc(16, 16, 1, &p->qh_bus);
    p->td = oa_host_dma_alloc(16, 16, 1, &p->td_bus);
    p->buf = oa_host_dma_alloc(1024, 64, 1, &p->buf_bus);
    if (p->qh == 0 || p->td == 0 || p->buf == 0) {
        return -1;
    }
    nuintrs++;
    p->len = pipe->len;
    p->ep = &d->in[pipe->ep & 15u];
    p->low = d->speed == USB_SPEED_LOW;
    p->token = PID_IN | ((oa_u32)d->addr << 8) | ((oa_u32)(pipe->ep & 15u) << 15) |
               ((oa_u32)(p->len - 1u) << 21);
    p->qh[0] = u->frames[0];               /* on to the others */
    intr_arm(p);
    for (i = 0; i < 1024u; i++) {
        u->frames[i] = (oa_u32)p->qh_bus | 2u;
    }
    oa_host_mb();
    pipe->hcpriv = p;
    return 0;
}

static int u_intr_poll(struct usb_hc *hc, struct usb_intr *pipe, void *buf, oa_u16 *actual)
{
    struct uintr *p = pipe->hcpriv;
    oa_u32 cs = p->td->cs, got;

    (void)hc;
    if (cs & TD_ACTIVE) {
        return 0;
    }
    oa_host_mb();
    if (cs & TD_ERRORS) {
        if (cs & TD_STALLED) {
            p->ep->toggle = 0;
        }
        intr_arm(p);
        return (cs & TD_STALLED) ? -USB_STALL : -USB_ERROR;
    }
    got = (cs + 1u) & 0x7FFu;
    if (got > p->len) {
        got = 0; /* null packet (7FFh) */
    }
    p->ep->toggle ^= 1u;
    *actual = (oa_u16)got;
    memcpy(buf, p->buf, got);
    intr_arm(p);
    return 1;
}

static const struct usb_hc_ops uhci_ops = {
    u_ports, u_port_reset, u_address, 0, 0, u_control, u_transfer, 0, 0, u_intr_open, u_intr_poll
};

static void hex(char *out, oa_u32 v, unsigned digits)
{
    while (digits--) {
        out[digits] = "0123456789abcdef"[v & 15u];
        v >>= 4;
    }
}

int usb_add_uhci(oa_u32 pci)
{
    struct uhc *u;
    oa_u32 bar, cmd, *fl;
    oa_u64 fl_bus, end;
    unsigned i;

    if (nuhcs == MAX_CONTROLLERS || !oa_host_has_port_io() ||
        (oa_host_pci_read32(pci, 0x08) >> 8) != ((oa_u32)USB_PCI_CLASS << 8 | USB_PCI_UHCI)) {
        return -1;
    }
    bar = oa_host_pci_read32(pci, 0x20); /* BAR4, I/O */
    if (!(bar & 1u) || (bar & 0xFFE0u) == 0u) {
        return -1;
    }
    cmd = oa_host_pci_read32(pci, 0x04);
    oa_host_pci_write32(pci, 0x04, (cmd & 0xFFFFu) | 0x0005u); /* I/O, bus master */
    /* legacy support off: clear the SMI status, no keyboard emulation */
    oa_host_pci_write32(pci, 0xC0, (oa_host_pci_read32(pci, 0xC0) & 0xFFFF0000ul) | 0x8F00u);

    u = &uhcs[nuhcs];
    memset(u, 0, sizeof(*u));
    u->io = (oa_u16)(bar & 0xFFE0u);
    outw(u, R_CMD, 0);
    outw(u, R_INTR, 0);
    outw(u, R_CMD, CMD_GRESET);
    oa_host_delay_us(50000);
    outw(u, R_CMD, 0);
    oa_host_delay_us(10000);
    outw(u, R_CMD, CMD_HCRST);
    end = oa_host_time_us() + 50000u;
    while ((inw(u, R_CMD) & CMD_HCRST) && oa_host_time_us() < end) {
        oa_host_delay_us(100);
    }

    fl = oa_host_dma_alloc(PAGE, PAGE, 1, &fl_bus);
    u->qh = oa_host_dma_alloc(16, 16, 1, &u->qh_bus);
    u->td = oa_host_dma_alloc(16u * NTD, 16, 1, &u->td_bus);
    u->bounce = oa_host_dma_alloc(PAGE, PAGE, 1, &u->bounce_bus);
    if (fl == 0 || u->qh == 0 || u->td == 0 || u->bounce == 0) {
        return -1;
    }
    u->frames = fl;
    u->qh[0] = 1u; /* no next queue head */
    u->qh[1] = 1u; /* empty */
    for (i = 0; i < 1024u; i++) {
        fl[i] = (oa_u32)u->qh_bus | 2u;
    }
    oa_host_mb();
    oa_host_outl((oa_u16)(u->io + R_FLBASE), (oa_u32)fl_bus);
    outw(u, R_FRNUM, 0);
    oa_host_outb((oa_u16)(u->io + R_SOF), 0x40u);
    outw(u, R_STS, 0xFFFFu);
    outw(u, R_CMD, CMD_RS | CMD_CF | CMD_MAXP);
    oa_host_delay_us(100000);

    /* Interrupt-driven completion over legacy INTx (UHCI has no MSI): bind the
     * line via the ACPI _PRT and enable IOC/error. On any failure ev stays NULL
     * and the frame list is polled -- no regression. */
    u->ev = oa_host_event_create();
    if (u->ev && oa_host_pci_intx(pci, uhci_irq, u) >= 0) {
        outw(u, R_STS, STS_INT | STS_ERR);    /* clear stale */
        outw(u, R_INTR, INTR_IOC | INTR_TMO); /* arm completion + error */
    } else if (u->ev) {
        oa_host_event_destroy(u->ev);
        u->ev = 0;
    }

    memcpy(u->hc.name, "UHCI 0000:00:00.0", 18);
    hex(u->hc.name + 5, pci >> 16, 4);
    hex(u->hc.name + 10, (pci >> 8) & 0xFFu, 2);
    hex(u->hc.name + 13, (pci >> 3) & 0x1Fu, 2);
    u->hc.name[16] = (char)('0' + (pci & 7u));
    u->hc.ops = &uhci_ops;
    u->hc.priv = u;
    u->hc.max_xfer = PAGE;
    nuhcs++;
    usb_hc_register(&u->hc);
    return 0;
}
