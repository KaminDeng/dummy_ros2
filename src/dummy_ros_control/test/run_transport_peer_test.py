"""Own the localhost D06 process and clean it up on every test exit."""
import socket
import subprocess
import sys
import time


def main():
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
