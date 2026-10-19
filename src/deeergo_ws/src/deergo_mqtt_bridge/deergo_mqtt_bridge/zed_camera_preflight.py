"""Reset only an idle first-generation ZED before the SDK opens its stream."""

import argparse
import fcntl
import os
import subprocess
from pathlib import Path

USBDEVFS_RESET = (ord("U") << 8) | 20


def reset_zed(sysfs_root=Path("/sys/bus/usb/devices"), usb_root=Path("/dev/bus/usb")):
    for device in sorted(sysfs_root.iterdir()):
        try:
            if (device / "idVendor").read_text().strip() != "2b03":
                continue
            if (device / "idProduct").read_text().strip() != "f580":
                continue
            bus = int((device / "busnum").read_text())
            number = int((device / "devnum").read_text())
        except (OSError, ValueError):
            continue

        video_nodes = [
            "/dev/" + video.name for video in device.glob("*/video4linux/video*")
        ]
        usb_device = usb_root / f"{bus:03d}" / f"{number:03d}"
        # The SDK may capture directly through libusb instead of /dev/video*.
        try:
            busy = subprocess.run(
                ["fuser", str(usb_device), *video_nodes],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            print(
                f"ZED preflight skipped: cannot check camera ownership: {error}",
                flush=True,
            )
            return False
        if busy.returncode != 1:
            print(
                "ZED preflight skipped: camera is in use or ownership is unknown.",
                flush=True,
            )
            return False

        try:
            descriptor = os.open(usb_device, os.O_WRONLY)
            try:
                fcntl.ioctl(descriptor, USBDEVFS_RESET)
            finally:
                os.close(descriptor)
        except OSError as error:
            print(
                f"ZED USB preflight failed: {error}. "
                "Install the camera USB rule with scripts/ros2_env.sh.",
                flush=True,
            )
            return False
        print(f"ZED USB ready: reset only {usb_device} (2b03:f580).", flush=True)
        return True

    print("ZED USB preflight: no first-generation ZED connected.", flush=True)
    return False


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-model", default="zed")
    options = parser.parse_args(args)
    if options.camera_model == "zed":
        reset_zed()
    # A preflight failure must not terminate SLAM; the SDK still reports its status.


if __name__ == "__main__":
    main()
