"""Interactive Bluetooth loopback tester for the Android tablet.

Run on the Pi:   python3 tools/bt_echo.py

Waits for the tablet to connect, then anything typed here is sent to the tablet
and anything the tablet sends back is printed. Use it to prove the round trip
works before running task1_main.py.

Type a message as either
    hello world            -> {"cat": "info", "value": "hello world"}
    status:running         -> {"cat": "status", "value": "running"}
Ctrl-C or "quit" to stop.
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import threading
import time

from communication.android import AndroidLink, AndroidMessage

link = AndroidLink()
stop = threading.Event()


def receiver():
    """Print everything the tablet sends, until the link drops."""
    while not stop.is_set():
        try:
            msg = link.recv()
        except OSError:
            if not stop.is_set():
                print("\n[!] tablet disconnected")
                stop.set()
            return
        if msg is None or msg == "":
            print("\n[!] tablet closed the connection")
            stop.set()
            return
        # A single recv can carry several newline-separated messages.
        for line in msg.splitlines():
            if line.strip():
                print(f"\n<-- FROM TABLET: {line.strip()}")
        print("send> ", end="", flush=True)


def main():
    print("Waiting for the tablet to connect on Bluetooth...")
    print("(pair with MDPGrp24 first, then connect from the MDP app)\n")
    link.connect()
    print("\n=== CONNECTED ===")
    print('Type a message and press Enter. Use "cat:value" to set a category.')
    print('Try: image-rec:{"image_id": "A", "obstacle_id": "1"}\n')

    threading.Thread(target=receiver, daemon=True).start()

    try:
        while not stop.is_set():
            try:
                raw = input("send> ").strip()
            except EOFError:
                break
            if not raw:
                continue
            if raw.lower() in ("quit", "exit"):
                break

            # "cat:value" if the part before the first colon looks like a category
            cat, value = "info", raw
            if ":" in raw:
                head, tail = raw.split(":", 1)
                if head and " " not in head and "{" not in head:
                    cat, value = head, tail

            message = AndroidMessage(cat, value)
            try:
                link.send(message)
                print(f"--> SENT: {message.jsonify}")
            except OSError as e:
                print(f"[!] send failed: {e}")
                break
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        stop.set()
        link.disconnect()
        print("link closed")


if __name__ == "__main__":
    main()
