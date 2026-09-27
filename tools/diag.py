"""Read the usb2spdif diagnostic string from a running device.

The firmware serves a hidden USB string descriptor (index 0x20) with the
previous-boot crash record and live status. It is read through the
picotool reset interface, which Windows binds to WinUSB automatically.

    pip install pyusb libusb-package
    python tools/diag.py            # one reading
    python tools/diag.py 10         # ten readings, one per second

Output fields:
    b<N>        boot count since power-up
    none/...    how the previous boot ended: none, panic, fault, hang, wdog
    m<N>        (after a crash) last breadcrumb, see DIAG_MARK_* in src/diag/diag.h
    pc / lr     (after a crash) code address of the panic message, fault or hang
    |           everything after the last "| " is the application status:
    D/I/S       link state: detached, idle, streaming
    a/o         host detected as Apple (3-byte 10.14 feedback) or other
    rate/bits   current stream format
    ff<N>       receive FIFO level in bytes (depth 9312, held half full)
    ur<N>       underruns: output ring ran dry while playing
    run0/1      stream samples going out on S/PDIF
    led0/1      onboard LED state
    tx<Hz>      S/PDIF output sample rate, 0 when the output is stopped
    ov<N>       overruns: USB packets that overwrote unread FIFO data
    lt<N>       S/PDIF interrupts too late to refill a buffer in time
    rmin<N>     lowest output ring level while playing, in frames (target 192)
    fmax<N>     highest receive FIFO level in bytes
    lp<us>      longest main-loop iteration in microseconds
                (~1000 at 96 kHz loses a USB packet)
    ur, ov, lt and rmin count from the start of the current or last stream
    (open or format change); fmax and lp from when it began playing, so the
    start-up (pre-roll, clock switch) is excluded. Counters stop at 999999.
"""
import sys
import time

import libusb_package
import usb.core
import usb.util

VID, PID = 0x1209, 0x0001
STRID_DIAG = 0x20
LANGID_EN_US = 0x0409


def read_diag(backend, index=STRID_DIAG):
    dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
    if dev is None:
        return "<device not found>"
    try:
        return usb.util.get_string(dev, index, LANGID_EN_US)
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
