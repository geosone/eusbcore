# eusbcore

eusbcore is a USB host stack in C90 for 32- and 64-bit systems that drive the controllers themselves, such as the GDOS kernel of [GonWare](https://gitea.disconnected-by-peer.at/GonSoft/GonWare). It follows the DOS driver [EUSBASPI.SYS](https://github.com/geosone/eusbaspi) and runs on the host services of [OpenASPI](https://github.com/geosone/OpenASPI)'s native layer (`openaspi/native/host.h`: DMA memory, MMIO, PCI configuration space, port I/O, time, log). Everything is polled.

| File | |
|---|---|
| `src/usb.c` | enumeration: root ports, addresses, descriptors, configuration; hubs (up to five tiers, USB 3 hubs not yet); route string and transaction translator for each device; `usb_add()` |
| `src/xhci.c` | xHCI: BIOS handoff, scratchpads, command and event ring, a slot and transfer rings per device |
| `src/ehci.c` | EHCI: BIOS handoff, async schedule, a queue head per endpoint; full- and low-speed devices on root ports go to the companion controller, behind a high-speed hub they use split transactions |
| `src/ohci.c` | OHCI: ownership from SMM, control and bulk lists, an ED per endpoint |
| `src/uhci.c` | UHCI: legacy support off, one queue head in every frame, a TD per packet (port I/O) |

Class drivers on top of it:

- [EUSBASPI](https://github.com/geosone/eusbaspi) `native/`: mass storage (Bulk-Only, CBI/CB) as an ASPI adapter
- [openinput](https://github.com/geosone/openinput): keyboards and mice (USB HID, PS/2, serial, virtio-input)

## Use

The embedding system implements OpenASPI's host services, adds its controllers with `usb_add()` (EHCI before its companions), calls `usb_enumerate()` and hands the devices (`usb_device_count()`, `usb_device()`) to the class drivers, which use `usb_control()` and `usb_transfer()`.

CMake builds it as `EUSBCORE::core` on top of `OpenASPI::host`; a parent project provides that target, otherwise `OPENASPI_SOURCE_DIR` or an OpenASPI checkout next to this one is used:

```sh
cmake -S . -B build -G Ninja && ninja -C build
```

Class drivers include eusbcore as a submodule and use the parent's `EUSBCORE::core` when there is one, so a system with several class drivers builds the core once.

Tested in QEMU on x86-64, i686 and AArch64 through GDOS: xHCI (super speed, a hub), EHCI, OHCI, UHCI and an EHCI with UHCI companions and a full-speed hub.

## License

GPL version 2 or later, as EUSBASPI.
