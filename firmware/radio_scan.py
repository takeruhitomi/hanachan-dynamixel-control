"""Find the UART baud rate of the OpenRB wireless link.

Both radio modules are transparent, so neither end answers a query of its own.
The rate can only be found by trying combinations until a known pattern makes
it across intact. This drives both halves of that search: the OpenRB side over
USB (which always works), the PC side over the dongle.

Flash firmware/openrb_radio_probe to the OpenRB first, then:

    uv run python firmware/radio_scan.py --usb COM6 --radio COM7

The DYNAMIXEL bus is untouched, so no servo can move during the scan.
"""

import argparse
import time

import serial

CANDIDATES = (9600, 19200, 38400, 57600, 115200, 230400, 460800, 1000000)
MAGIC = b"HANA?HANA?HANA?HANA?"


def probe_command(usb: serial.Serial, command: str, timeout: float = 2.0) -> str:
    usb.reset_input_buffer()
    usb.write((command + "\n").encode("ascii"))
    usb.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = usb.readline().decode("ascii", errors="replace").strip()
        if line:
            return line
    return ""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usb", default="COM6", help="OpenRB USB port")
    parser.add_argument("--radio", default="COM7", help="PC-side radio dongle port")
    parser.add_argument("--settle", type=float, default=0.6)
    args = parser.parse_args()

    with serial.Serial(args.usb, 115200, timeout=0.4) as usb:
        time.sleep(1.0)  # let the USB CDC come up
        usb.reset_input_buffer()
        reply = probe_command(usb, "?")
        if "baud" not in reply:
            print(f"probe firmware did not answer on {args.usb} (got {reply!r})")
            print("flash firmware/openrb_radio_probe first")
            return 1
        print(f"probe firmware ready on {args.usb}: {reply}")

        matches = []
        print(f"\n{'OpenRB':>9}  {'PC':>9}  result")
        for openrb_baud in CANDIDATES:
            set_reply = probe_command(usb, f"B{openrb_baud}")
            if not set_reply.startswith("OK"):
                print(f"{openrb_baud:>9}  {'-':>9}  cannot set: {set_reply}")
                continue

            for pc_baud in CANDIDATES:
                try:
                    with serial.Serial(args.radio, pc_baud, timeout=0.2) as radio:
                        radio.reset_output_buffer()
                        radio.write(MAGIC)
                        radio.flush()
                except serial.SerialException as exc:
                    print(f"{openrb_baud:>9}  {pc_baud:>9}  port error: {exc}")
                    continue

                time.sleep(args.settle)
                report = probe_command(usb, "R")
                got_clean = "ASCII " in report and "HANA?" in report.split("ASCII ", 1)[1]
                byte_count = 0
                if "BYTES " in report:
                    try:
                        byte_count = int(report.split("BYTES ", 1)[1].split()[0])
                    except (IndexError, ValueError):
                        byte_count = 0

                if got_clean:
                    print(f"{openrb_baud:>9}  {pc_baud:>9}  MATCH  {report}")
                    matches.append((openrb_baud, pc_baud))
                elif byte_count:
                    print(f"{openrb_baud:>9}  {pc_baud:>9}  {byte_count} garbled bytes")

        print()
        if not matches:
            print("no combination carried the pattern intact.")
            print("check that the module is on the 4-pin Serial2 port and that both")
            print("radios are powered and paired.")
            return 1

        print("working combinations (OpenRB baud, PC baud):")
        for openrb_baud, pc_baud in matches:
            print(f"  {openrb_baud} / {pc_baud}")
        openrb_baud, pc_baud = matches[0]
        print(f"\nset RADIO_BAUD = {openrb_baud} in openrb_wireless_bridge.ino,")
        print(f"then drive the robot with:  --device {args.radio} --baudrate {pc_baud}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
