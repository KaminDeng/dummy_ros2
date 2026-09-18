"""Own the localhost D06 process and clean it up on every test exit."""
import socket
import subprocess
import sys
import time
from pathlib import Path


CDC_GLOB = "/dev/serial/by-id/usb-Dummy_Robotics_STM32_Virtual_ComPort_in_FS_Mode_*-if00"


def resolve_real_endpoint(environment, globber=Path("/").glob):
    """Resolve only the device CDC port; J-Link VCP is never an eligible peer."""
    explicit = environment.get("DUMMY_ROS_PRODUCTION_ENDPOINT")
    if explicit:
        if "/ttyACM0" in explicit or not explicit.startswith("serial//dev/"):
            raise RuntimeError("DUMMY_ROS_PRODUCTION_ENDPOINT must name the Dummy Robotics CDC endpoint")
        return explicit
    ports = sorted(globber(CDC_GLOB.lstrip("/")))
    if len(ports) != 1:
        raise RuntimeError(f"expected exactly one Dummy Robotics CDC device, found {len(ports)}")
    return f"serial/{ports[0]}#baudrate=115200"


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--print-real-endpoint":
        print(resolve_real_endpoint(__import__("os").environ))
        return 0
    if len(sys.argv) != 3:
        raise RuntimeError("usage: run_transport_peer_test.py PEER TEST")
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    endpoint = f"tcp/127.0.0.1:{port}"
    peer = subprocess.Popen([sys.argv[1], "--listen", endpoint],
                            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if peer.poll() is not None:
                raise RuntimeError(f"D06 peer exited: {peer.stderr.read().decode()}")
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                    break
            except OSError:
                time.sleep(0.02)
        else:
            raise RuntimeError("D06 peer did not listen within 10 seconds")
        return subprocess.run([sys.argv[2], endpoint], timeout=20, check=False).returncode
    finally:
        peer.terminate()
        try:
            peer.wait(timeout=5)
        except subprocess.TimeoutExpired:
            peer.kill()
            peer.wait(timeout=5)


if __name__ == "__main__":
    sys.exit(main())
