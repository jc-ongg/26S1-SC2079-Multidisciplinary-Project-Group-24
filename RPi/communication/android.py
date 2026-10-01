import json
import re
import socket
import subprocess
from collections import deque
from typing import Optional
from communication.link import Link
from communication.android_protocol import AndroidState, FrameBuffer, to_frame

# RFCOMM channel the Pi listens on. The SPP record published with sdptool points
# here, and Android apps that skip SDP lookup can connect to this channel directly.
RFCOMM_CHANNEL = 1


def _run(cmd: list) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True)


class AndroidMessage:
    """
    Class for communicating with Android tablet over Bluetooth connection.
    """

    def __init__(self, cat: str, value: str):
        """
        Constructor for AndroidMessage.
        :param cat: Message category.
        :param value: Message value.
        """
        self._cat = cat
        self._value = value

    @property
    def cat(self):
        """
        Returns the message category.
        :return: String representation of the message category.
        """
        return self._cat

    @property
    def value(self):
        """
        Returns the message as a string.
        :return: String representation of the message.
        """
        return self._value

    @property
    def jsonify(self) -> str:
        """
        Returns the message as a JSON string.
        :return: JSON string representation of the message.
        """
        return json.dumps({'cat': self._cat, 'value': self._value})


class AndroidLink(Link):
    """Class for communicating with Android tablet over Bluetooth connection.

    Uses the standard library's AF_BLUETOOTH socket rather than pybluez, which
    cannot be built on this Pi: pybluez needs distutils (removed in Python 3.12+)
    and libbluetooth headers that aren't installed. The SPP service record is
    published with sdptool, which needs bluetoothd running in --compat mode.

    ## Wire format

    The tablet speaks the Group 24 protocol (MDP_Group24_Bluetooth_Protocol.md):
    ``//<payload>//`` frames, no newline, e.g. ``//OBSTACLE,1,5,7,E//``,
    ``//BEGIN,1//``, ``//F//``. android_protocol.py does the framing and
    translation, so recv() still hands final_week_8.py the old
    ``{"cat": ..., "value": ...}`` JSON and send() still takes AndroidMessage:

        tablet -> RPi                     what recv() returns
        //OBSTACLE,..// //ROBOT,..//      (nothing; stored until BEGIN)
        //BEGIN,1//                       {"cat":"obstacles",...} then
                                          {"cat":"control","value":"start"}
        //F// //B// //L// //R//           {"cat":"manual","value":"FW010"} etc.
        //MAP//                           (nothing; wipes the stored map)

        AndroidMessage                    frame sent
        ("location", {x,y,d})             //ROBOT,8,12,N//
        ("image-rec", {obstacle_id, image_id})   //TARGET,3,11//
        ("info"/"status"/"error", text)   //STATUS,text//
    """

    def __init__(self):
        """
        Initialize the Bluetooth connection.
        """
        super().__init__()
        self.client_sock = None
        self.server_sock = None
        self.channel = None
        # Bluetooth is a byte stream: a read may carry a partial frame or
        # several, so framing state has to persist across recv() calls.
        self._frames = FrameBuffer()
        self._state = AndroidState()
        self._pending = deque()

    def _make_discoverable(self):
        """Make the adapter visible and accepting pairings.

        discoverable-timeout must be 0 first, or BlueZ hides the adapter again
        after its default 180 seconds and the tablet stops seeing it.
        """
        for args in (["power", "on"], ["discoverable-timeout", "0"],
                     ["discoverable", "on"], ["pairable", "on"]):
            result = _run(["bluetoothctl"] + args)
            if result.returncode != 0:
                self.logger.warning(
                    f"bluetoothctl {' '.join(args)} failed: {result.stderr.strip()}")

    def _trust_paired_devices(self):
        """Trust every paired device so reconnects need no confirmation prompt."""
        result = _run(["bluetoothctl", "devices", "Paired"])
        if result.returncode != 0 or not result.stdout.strip():
            # Older BlueZ spells it differently.
            result = _run(["bluetoothctl", "paired-devices"])
        for mac in re.findall(r"Device\s+([0-9A-F:]{17})", result.stdout, re.I):
            _run(["bluetoothctl", "trust", mac])
            self.logger.info(f"Trusted paired device {mac}")

    def _clear_sp_records(self):
        """Remove Serial Port records left behind by earlier runs.

        Each run publishes a fresh record; without this they pile up, and the
        tablet may pick a stale one pointing at a channel nobody listens on.
        """
        result = _run(["sdptool", "browse", "local"])
        handles = []
        for block in result.stdout.split("\n\n"):
            if "Serial Port" not in block:
                continue
            m = re.search(r"Service RecHandle:\s*(0x[0-9a-fA-F]+)", block)
            if m:
                handles.append(m.group(1))
        for h in handles:
            _run(["sdptool", "del", h])
        if handles:
            self.logger.info(
                f"Removed {len(handles)} stale SPP record(s): {handles}")

    def _advertise(self):
        """Publish a Serial Port Profile record pointing at our RFCOMM channel."""
        self._clear_sp_records()
        result = _run(["sdptool", "add", f"--channel={self.channel}", "SP"])
        if result.returncode != 0:
            self.logger.warning(
                "sdptool could not publish the SPP record "
                f"({result.stderr.strip() or 'unknown error'}). Is bluetoothd in "
                "--compat mode and /var/run/sdp writable? Android apps that "
                f"connect straight to channel {self.channel} still work.")
        else:
            self.logger.info(f"Advertised SPP service on channel {self.channel}")

    def connect(self):
        """
        Connect to Andriod by Bluetooth
        """
        self.logger.info("Bluetooth connection started")
        try:
            self._make_discoverable()
            self._trust_paired_devices()

            self.server_sock = socket.socket(
                socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            self.server_sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Prefer the advertised channel, but step forward if it's taken by a
            # previous run that hasn't released it yet. If this lands on 2+,
            # an old process is still alive -- kill it, the tablet expects 1.
            for channel in range(RFCOMM_CHANNEL, RFCOMM_CHANNEL + 10):
                try:
                    self.server_sock.bind((socket.BDADDR_ANY, channel))
                    self.channel = channel
                    break
                except OSError:
                    continue
            if self.channel is None:
                raise OSError("No free RFCOMM channel to bind to")

            self.server_sock.listen(1)
            self._advertise()

            self.logger.info(
                f"Awaiting Bluetooth connection on RFCOMM CHANNEL {self.channel}")
            self.client_sock, client_info = self.server_sock.accept()
            self.logger.info(f"Accepted connection from: {client_info}")

        except Exception as e:
            self.logger.error(f"Error in Bluetooth link connection: {e}")
            self.disconnect()
            raise

    def disconnect(self):
        """Disconnect from Android Bluetooth connection and shutdown all the sockets established"""
        self.logger.debug("Disconnecting Bluetooth link")
        for name in ("client_sock", "server_sock"):
            sock = getattr(self, name)
            if sock is None:
                continue
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError as e:
                self.logger.error(f"Failed to close {name}: {e}")
            setattr(self, name, None)
        self.channel = None
        self.logger.info("Disconnected Bluetooth link")

    def send(self, message: AndroidMessage):
        """Send a message to Android as a //FRAME//.

        to_frame() returns None for messages with nothing useful to show (an
        image-rec result of "NA", for instance) -- those are skipped rather
        than sent as an empty frame.
        """
        payload = to_frame(message.cat, message.value)
        if payload is None:
            self.logger.debug(f"Nothing to send for {message.cat}: {message.value}")
            return
        try:
            self.client_sock.send(payload.encode("utf-8"))
            self.logger.debug(f"Sent to Android: {payload}")
        except OSError as e:
            self.logger.error(f"Error sending message to Android: {e}")
            raise e

    def recv(self) -> Optional[str]:
        """Return the next Android message as a JSON string.

        Frames that only update state (ROBOT/OBSTACLE/MAP) produce no message,
        so this keeps reading until something actionable arrives. The return
        value stays a JSON string of {"cat": ..., "value": ...} so
        recv_android() in final_week_8.py is unchanged.
        """
        while not self._pending:
            try:
                tmp = self.client_sock.recv(1024)
            except OSError as e:
                self.logger.error(f"Error receiving message from Android: {e}")
                raise e
            if not tmp:
                raise OSError("Android closed the connection")
            self.logger.debug(f"Raw from Android: {tmp!r}")
            for payload in self._frames.feed(tmp.decode("utf-8", errors="replace")):
                self.logger.debug(f"Frame from Android: {payload}")
                for msg in self._state.handle(payload):
                    self._pending.append(msg)

        message = self._pending.popleft()
        self.logger.debug(f"Received from Android: {message}")
        return json.dumps(message)
