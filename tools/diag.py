"""Read the usb2spdif diagnostic string from a running device.

The firmware serves a hidden USB string descriptor (index 0x20) with the
previous-boot crash record and live status. It is read through the
picotool reset interface, which Windows binds to WinUSB automatically.

    pip install pyusb libusb-package
    python tools/diag.py            # one reading
    python tools/diag.py 10         # ten readings, one per second

Output fields:
    b<N>        boot count since power-up
    prev=...    how the previous boot ended: none, panic, fault, hang, wdog
    m<N>        last breadcrumb before that (see DIAG_MARK_* in src/diag/diag.h)
    pc / lr     code address of the panic message, fault or hang
    I<N> C<N>   SET_INTERFACE and clock requests seen this boot
    D/I/S       link state: detached, idle, streaming
    a/o         host detected as Apple (3-byte 10.14 feedback) or other
    rate/bits   current stream format
    ff<a>/<b>   receive FIFO level / depth in bytes
    ur<N>       sink underruns
    run0/1      sink running (audio flowing)
    led0/1      onboard LED state
"""
import sys
import time

import libusb_package
import usb.core
import usb.util

VID, PID = 0x1209, 0x0001
STRID_DIAG = 0x20
LANGID_EN_US = 0x0409


def read_diag(backend):
    dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
    if dev is None:
        return "<device not found>"
    try:
        return usb.util.get_string(dev, STRID_DIAG, LANGID_EN_US)
    except usb.core.USBError as e:
        return f"<error: {e}>"
    finally:
        usb.util.dispose_resources(dev)


def main():
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    backend = libusb_package.get_libusb1_backend()
    for i in range(count):
        print(time.strftime("%H:%M:%S"), read_diag(backend))
        if i + 1 < count:
            time.sleep(1)


if __name__ == "__main__":
    main()
