/* SPDX-License-Identifier: GPL-2.0-or-later */
/* OHCI host controller for eusbcore, the native USB stack (polled).
 *
 * As EUSBASPI.SYS's ohci_init: ownership from SMM (OwnershipChangeRequest),
 * reset with the frame interval kept, control and bulk lists only. Each
 * endpoint has an ED that stays in its list; a transfer hangs general TDs
 * between HeadP and a dummy tail and is polled until HeadP reaches the
 * tail or the ED halts. Interrupt endpoints (CBI status) go on the bulk
 * list. A transfer is at most one page, so one TD never crosses a page
 * and a short packet cannot leave TDs behind. */

#include <string.h>

#include "eusbcore/usb.h"
#include "openaspi/native/host.h"

#define MAX_CONTROLLERS 4
#define MAX_ED          64
#define NTD             4u
#define PAGE            4096u

#define HC_CONTROL   0x04u
#define HC_CMDSTATUS 0x08u
#define HC_INTSTATUS 0x0Cu
#define HC_INTDIS    0x14u
#define HC_HCCA      0x18u
#define HC_CTRLHEAD  0x20u
#define HC_CTRLCUR   0x24u
#define HC_BULKHEAD  0x28u
#define HC_BULKCUR   0x2Cu
#define HC_FMINTERVAL 0x34u
#define HC_PERIODICSTART 0x40u
#define HC_RHDESCA   0x48u
#define HC_RHSTATUS  0x50u
#define HC_RHPORT    0x54u

#define CTL_CLE  0x10u
#define CTL_BLE  0x20u
#define CTL_HCFS 0xC0u
#define CTL_OPER 0x80u
#define CTL_IR   0x100u
#define CS_HCR   0x01u
#define CS_CLF   0x02u
#define CS_BLF   0x04u
#define CS_OCR   0x08u

#define RH_CCS  (1ul << 0)
#define RH_PES  (1ul << 1)
#define RH_PRS  (1ul << 4)
#define RH_PPS  (1ul << 8)
#define RH_LSDA (1ul << 9)
#define RH_CSC  (1ul << 16)
#define RH_PRSC (1ul << 20)

#define TD_SETUP 0u
#define TD_OUT   1u
#define TD_IN    2u
#define CC_NOERR 0u
#define CC_STALL 4u
#define CC_NOTACCESSED 0xFu

struct ed {
    oa_u32 info, tail, head, next;
};

struct td {
    oa_u32 info, cbp, next, be;
};

struct pipe {
    struct usb_dev *dev;
    oa_u8 ep;
    volatile struct ed *ed;
    oa_u64 ed_bus;
};

struct ohc {
    struct usb_hc hc;
    volatile oa_u8 *regs;
    unsigned nports;
    volatile struct td *td;     /* NTD + 1 (the tail) */
    oa_u64 td_bus;
    oa_u8 *bounce;
    oa_u64 bounce_bus;
    struct pipe pipes[MAX_ED];
    unsigned npipes;
    volatile struct ed *ctrl_last, *bulk_last;
};

static struct ohc ohcs[MAX_CONTROLLERS];
static unsigned nohcs;

static oa_u32 rd(struct ohc *o, unsigned off)
{
    return *(volatile oa_u32 *)(o->regs + off);
}

static void wr(struct ohc *o, unsigned off, oa_u32 v)
{
    *(volatile oa_u32 *)(o->regs + off) = v;
}

static unsigned o_ports(struct usb_hc *hc)
{
    return ((struct ohc *)hc->priv)->nports;
}

static int o_port_reset(struct usb_hc *hc, unsigned port, oa_u8 *speed)
{
    struct ohc *o = hc->priv;
    unsigned off = HC_RHPORT + 4u * (port - 1u);
    oa_u64 end;
    oa_u32 v;

    if (!(rd(o, off) & RH_CCS)) {
        return -1;
    }
    wr(o, off, RH_PRS);
    end = oa_host_time_us() + 100000u;
    while (!(rd(o, off) & RH_PRSC)) {
        if (oa_host_time_us() > end) {
            return -1;
        }
        oa_host_delay_us(1000);
    }
    wr(o, off, RH_PRSC | RH_CSC);
    oa_host_delay_us(20000);
    v = rd(o, off);
    if ((v & (RH_CCS | RH_PES)) != (RH_CCS | RH_PES)) {
        return -1;
    }
    *speed = (v & RH_LSDA) ? USB_SPEED_LOW : USB_SPEED_FULL;
    return 0;
}

static int o_address(struct usb_hc *hc, struct usb_dev *dev)
{
    return usb_set_address(hc, dev);
}

static struct pipe *pipe_of(struct ohc *o, struct usb_dev *d, oa_u8 ep)
{
    struct pipe *p;
    unsigned i, mps;
    int control = (ep & 15u) == 0u;

    for (i = 0; i < o->npipes; i++) {
        if (o->pipes[i].dev == d && o->pipes[i].ep == ep) {
            p = &o->pipes[i];
            goto update;
        }
    }
    if (o->npipes == MAX_ED) {
        return 0;
    }
    p = &o->pipes[o->npipes];
    p->ed = oa_host_dma_alloc(16, 16, 1, &p->ed_bus);
    if (p->ed == 0) {
        return 0;
    }
    o->npipes++;
    p->dev = d;
    p->ep = ep;
    p->ed->tail = p->ed->head = 0;
    p->ed->next = 0;
    oa_host_mb();
    /* append to the control or bulk list */
    if (control) {
        if (o->ctrl_last) {
            o->ctrl_last->next = (oa_u32)p->ed_bus;
        } else {
            wr(o, HC_CTRLHEAD, (oa_u32)p->ed_bus);
        }
        o->ctrl_last = p->ed;
    } else {
        if (o->bulk_last) {
            o->bulk_last->next = (oa_u32)p->ed_bus;
        } else {
            wr(o, HC_BULKHEAD, (oa_u32)p->ed_bus);
        }
        o->bulk_last = p->ed;
    }
update:
    mps = control ? d->out[0].mps : (ep & 0x80u) ? d->in[ep & 15u].mps : d->out[ep & 15u].mps;
    /* direction from the TD; skip while idle is not needed (head == tail) */
    p->ed->info = d->addr | ((oa_u32)(ep & 15u) << 7) |
                  (d->speed == USB_SPEED_LOW ? (1ul << 13) : 0u) | ((oa_u32)mps << 16);
    return p;
}

static void fill(struct ohc *o, unsigned i, oa_u32 pid, oa_u64 bus, oa_u32 len, oa_u32 toggle)
{
    volatile struct td *t = &o->td[i];

    /* rounding (short packets are fine), no interrupt, toggle from the TD
     * (toggle < 2) or the ED (toggle == 2) */
    t->info = (1ul << 18) | (pid << 19) | (7ul << 21) |
              (toggle < 2u ? ((2ul | toggle) << 24) : 0u) | ((oa_u32)CC_NOTACCESSED << 28);
    t->cbp = len ? (oa_u32)bus : 0u;
    t->be = len ? (oa_u32)(bus + len - 1u) : 0u;
    t->next = (oa_u32)(o->td_bus + 16u * (i + 1u));
}

/* Hang td[0..n-1] (and the dummy tail td[n]) into the ED and poll. */
static int run(struct ohc *o, struct pipe *p, unsigned n, int control, oa_u32 timeout_ms)
{
    oa_u64 end = oa_host_time_us() + (oa_u64)timeout_ms * 1000u;
    oa_u64 tail = o->td_bus + 16u * n;
    oa_u32 cc, carry = p->ed->head & 2u;
    unsigned i;

    o->td[n].info = o->td[n].cbp = o->td[n].next = o->td[n].be = 0;
    p->ed->tail = (oa_u32)tail;
    oa_host_mb();
    p->ed->head = (oa_u32)o->td_bus | carry;
    oa_host_mb();
    wr(o, HC_CMDSTATUS, control ? CS_CLF : CS_BLF);
    for (;;) {
        if ((p->ed->head & ~0xFul) == (oa_u32)tail) {
            break;
        }
        if (p->ed->head & 1u) { /* halted */
            break;
        }
        if (oa_host_time_us() > end) {
            p->ed->info |= 1ul << 14; /* skip */
            oa_host_delay_us(2000);
            p->ed->head = (oa_u32)tail;
            p->ed->info &= ~(1ul << 14);
            return USB_TIMEOUT;
        }
    }
    oa_host_mb();
    for (i = 0; i < n; i++) {
        cc = o->td[i].info >> 28;
        if (cc == CC_NOERR) {
            continue;
        }
        if (cc >= 0xEu) {
            break; /* not reached: an earlier TD ended it */
        }
        /* error: clear the halt, keep the toggle carry */
        p->ed->head = (oa_u32)tail | (p->ed->head & 2u);
        return cc == CC_STALL ? USB_STALL : USB_ERROR;
    }
    if (p->ed->head & 1u) {
        p->ed->head = (oa_u32)tail | (p->ed->head & 2u);
        return USB_ERROR;
    }
    return USB_OK;
}

static oa_u32 td_bytes(struct ohc *o, unsigned i, oa_u32 len)
{
    oa_u32 cbp = o->td[i].cbp;

    if ((o->td[i].info >> 28) != CC_NOERR) {
        return 0;
    }
    if (cbp == 0u) {
        return len; /* all of it */
    }
    return len - (o->td[i].be - cbp + 1u);
}

static int o_control(struct usb_hc *hc, struct usb_dev *d, const oa_u8 setup[8], void *data,
                     oa_u16 len, oa_u16 *actual, oa_u32 timeout_ms)
{
    struct ohc *o = hc->priv;
    struct pipe *p = pipe_of(o, d, 0);
    int in = (setup[0] & 0x80u) != 0, r;
    unsigned n = 0;

    *actual = 0;
    if (p == 0 || len > PAGE - 64u) {
        return USB_ERROR;
    }
    memcpy(o->bounce, setup, 8);
    if (!in && len) {
        memcpy(o->bounce + 64, data, len);
    }
    fill(o, n++, TD_SETUP, o->bounce_bus, 8, 0);
    if (len) {
        fill(o, n++, in ? TD_IN : TD_OUT, o->bounce_bus + 64u, len, 1);
    }
    fill(o, n++, (len && in) ? TD_OUT : TD_IN, 0, 0, 1);
    r = run(o, p, n, 1, timeout_ms ? timeout_ms : 5000u);
    if (r != USB_OK) {
        return r;
    }
    if (len) {
        *actual = (oa_u16)td_bytes(o, 1, len);
        if (in && *actual) {
            memcpy(data, o->bounce + 64, *actual);
        }
    }
    return USB_OK;
}

static int o_transfer(struct usb_hc *hc, struct usb_dev *d, oa_u8 ep, void *data, oa_u32 len,
                      oa_u32 *actual, oa_u32 timeout_ms)
{
    struct ohc *o = hc->priv;
    struct pipe *p = pipe_of(o, d, ep);
    int in = (ep & 0x80u) != 0, r;

    *actual = 0;
    if (p == 0 || len > PAGE) {
        return USB_ERROR;
    }
    if (!in && len) {
        memcpy(o->bounce, data, len);
    }
    fill(o, 0, in ? TD_IN : TD_OUT, o->bounce_bus, len, 2); /* toggle carried by the ED */
    r = run(o, p, 1, 0, timeout_ms ? timeout_ms : 15000u);
    if (r == USB_OK) {
        *actual = td_bytes(o, 0, len);
        if (in && *actual) {
            memcpy(data, o->bounce, *actual);
        }
    }
    return r;
}

static void o_clear_halt(struct usb_hc *hc, struct usb_dev *d, oa_u8 ep)
{
    struct ohc *o = hc->priv;
    struct pipe *p = pipe_of(o, d, ep);

    if (p) {
        p->ed->head &= ~3ul; /* not halted, toggle DATA0 */
    }
}

static const struct usb_hc_ops ohci_ops = {
    o_ports, o_port_reset, o_address, 0, 0, o_control, o_transfer, o_clear_halt, 0
};

static void hex(char *out, oa_u32 v, unsigned digits)
{
    while (digits--) {
        out[digits] = "0123456789abcdef"[v & 15u];
        v >>= 4;
    }
}

int usb_add_ohci(oa_u32 pci)
{
    struct ohc *o;
    oa_u32 bar, cmd, fi, desca;
    oa_u64 end, hcca_bus;
    void *hcca;
    unsigned i;

    if (nohcs == MAX_CONTROLLERS ||
        (oa_host_pci_read32(pci, 0x08) >> 8) != ((oa_u32)USB_PCI_CLASS << 8 | USB_PCI_OHCI)) {
        return -1;
    }
    bar = oa_host_pci_read32(pci, 0x10) & 0xFFFFF000ul;
    if (bar == 0u) {
        return -1;
    }
    cmd = oa_host_pci_read32(pci, 0x04);
    oa_host_pci_write32(pci, 0x04, (cmd & 0xFFFFu) | 0x0006u);
    o = &ohcs[nohcs];
    memset(o, 0, sizeof(*o));
    o->regs = oa_host_map_mmio(bar, PAGE);
    if (o->regs == 0) {
        return -1;
    }
    /* take it from SMM */
    if (rd(o, HC_CONTROL) & CTL_IR) {
        wr(o, HC_CMDSTATUS, CS_OCR);
        end = oa_host_time_us() + 1000000u;
        while ((rd(o, HC_CONTROL) & CTL_IR) && oa_host_time_us() < end) {
            oa_host_delay_us(1000);
        }
    }
    wr(o, HC_INTDIS, 0xFFFFFFFFul);
    fi = rd(o, HC_FMINTERVAL);
    wr(o, HC_CMDSTATUS, CS_HCR);
    end = oa_host_time_us() + 10000u;
    while ((rd(o, HC_CMDSTATUS) & CS_HCR) && oa_host_time_us() < end) {
        oa_host_delay_us(10);
    }
    if ((fi & 0x3FFFu) == 0u) {
        fi = 0x2EDFu | (0x2778ul << 16); /* 12000 bit times, FSMPS */
    }
    wr(o, HC_FMINTERVAL, (fi & 0x7FFFFFFFul) ^ 0x80000000ul); /* toggle FIT */
    wr(o, HC_PERIODICSTART, (fi & 0x3FFFu) * 9u / 10u);

    hcca = oa_host_dma_alloc(256, 256, 1, &hcca_bus);
    o->td = oa_host_dma_alloc(16u * (NTD + 1u), 16, 1, &o->td_bus);
    o->bounce = oa_host_dma_alloc(PAGE, PAGE, 1, &o->bounce_bus);
    if (hcca == 0 || o->td == 0 || o->bounce == 0) {
        return -1;
    }
    memset(hcca, 0, 256);
    wr(o, HC_HCCA, (oa_u32)hcca_bus);
    wr(o, HC_CTRLHEAD, 0);
    wr(o, HC_BULKHEAD, 0);
    wr(o, HC_CTRLCUR, 0);
    wr(o, HC_BULKCUR, 0);
    wr(o, HC_INTSTATUS, 0xFFFFFFFFul);
    wr(o, HC_CONTROL, CTL_OPER | CTL_CLE | CTL_BLE | 3u); /* control:bulk 4:1 */

    desca = rd(o, HC_RHDESCA);
    o->nports = desca & 0xFFu;
    if (o->nports > 15u) {
        o->nports = 15;
    }
    wr(o, HC_RHSTATUS, 1ul << 16); /* global power */
    for (i = 0; i < o->nports; i++) {
        wr(o, HC_RHPORT + 4u * i, RH_PPS);
    }
    oa_host_delay_us(((desca >> 24) * 2u + 100u) * 1000u);

    memcpy(o->hc.name, "OHCI 0000:00:00.0", 18);
    hex(o->hc.name + 5, pci >> 16, 4);
    hex(o->hc.name + 10, (pci >> 8) & 0xFFu, 2);
    hex(o->hc.name + 13, (pci >> 3) & 0x1Fu, 2);
    o->hc.name[16] = (char)('0' + (pci & 7u));
    o->hc.ops = &ohci_ops;
    o->hc.priv = o;
    o->hc.max_xfer = PAGE;
    nohcs++;
    usb_hc_register(&o->hc);
    return 0;
}
