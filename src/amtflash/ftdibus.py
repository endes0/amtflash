import usb.core
import usb.util
from usb.util import build_request_type, CTRL_OUT, CTRL_IN, CTRL_TYPE_VENDOR, CTRL_RECIPIENT_DEVICE

class FTDIBus:
    """The low level FTDI bus interface to communicate with the AMT flasher
    """

    REQ_OUT = build_request_type(
        CTRL_OUT, CTRL_TYPE_VENDOR, CTRL_RECIPIENT_DEVICE)
    REQ_IN = build_request_type(
        CTRL_IN, CTRL_TYPE_VENDOR, CTRL_RECIPIENT_DEVICE)
    FRAC_DIV_CODE = (0, 3, 2, 4, 1, 5, 6, 7)

    # Values of the FTDI reset request
    RESET_SIO = 0
    PURGE_RX = 1
    PURGE_TX = 2

    READ_SIZE = 4 << 10

    class Parity(int):
        """Parity"""
        NONE = 0x00 << 8,
        ODD = 0x01 << 8,
        EVEN = 0x02 << 8,
        MARK = 0x03 << 8,
        SPACE = 0x04 << 8

    class StopBits(int):
        """Stop bits"""
        ONE = 0x00 << 11,
        ONE_POINT_FIVE = 0x01 << 11,
        TWO = 0x02 << 11

    def __init__(self, vid=0x1c43, pid=0x0500):
        self._vid = vid
        self._pid = pid
        self._in_ep = None   # device -> host (responses, received frames, K-line bytes)
        self._out_ep = None  # host -> device (commands)
        self._packet_size = 64
        self._write_bitmask = None
        self._read_bitmask = None
        self._pending = bytearray()
        # On the K-line the FTDI status may report framing/break errors while the line is held low by an init.
        # Outside of it a line error means a broken link.
        self.line_errors_fatal = True
        self.dev = None

    def open(self):
        """Open device"""
        self.dev: usb.core.Device = usb.core.find(
            idVendor=self._vid, idProduct=self._pid)
        if self.dev is None:
            raise RuntimeError('Device not found')

        self.dev.reset()
        self.dev.set_configuration()

        interface = self.dev.get_active_configuration()[(0, 0)]
        for endpoint in interface:
            if usb.util.endpoint_direction(endpoint.bEndpointAddress) == usb.util.ENDPOINT_IN:
                self._in_ep = endpoint.bEndpointAddress
                self._packet_size = endpoint.wMaxPacketSize
            else:
                self._out_ep = endpoint.bEndpointAddress
        if self._in_ep is None or self._out_ep is None:
            raise RuntimeError('Bulk endpoints not found')

        self.reset()
        self.set_latency_timer(1)

    def close(self):
        """Close device"""
        usb.util.dispose_resources(self.dev)

    def reset(self):
        """Reset the SIO and purge both buffers.

        Resetting the SIO alone does not empty the buffers: leftovers of a previous session come back as the
        answer to the next handshake, and opening fails every other run.
        """
        self.dev.ctrl_transfer(self.REQ_OUT, 0x00, self.RESET_SIO, 0x00, None)
        self.dev.ctrl_transfer(self.REQ_OUT, 0x00, self.PURGE_RX, 0x00, None)
        self.dev.ctrl_transfer(self.REQ_OUT, 0x00, self.PURGE_TX, 0x00, None)
        self._pending.clear()

    def read_EE(self, addr: int, size: int) -> bytes:
        """Read EEPROM

        Args:
            addr (int): eeprom address to read from
            size (int): size of data to read

        Returns:
            bytes: data read from eeprom
        """
        data = bytearray()
        for i in range(0, size, 2):
            temp = self.dev.ctrl_transfer(
                self.REQ_IN, 0x90, 0x0, addr+i, 2)
            data += temp
        return data

    def write_EE(self, addr: int, data: bytes):
        """Write EEPROM

        Args:
            addr (int): eeprom address to write to
            data (bytes): data to write
        """
        self.dev.ctrl_transfer(self.REQ_OUT, 0x91, 0x0, addr, data)

    def write(self, data: bytes):
        """Write to the Serial interface (commands)

        Args:
            data (bytes): data to write
        """
        # Apply write bitmask
        if self._write_bitmask is not None:
            data = bytearray(data)
            for i in range(len(data)):
                data[i] ^= self._write_bitmask
        self.dev.write(self._out_ep, data)

    def _unpack(self, chunk) -> bytearray:
        """Payload of a bulk IN transfer, unmasked.

        The FTDI puts 2 status bytes at the start of **every** packet of `wMaxPacketSize` bytes, and a single
        transfer carries several packets when data keeps coming (e.g. CAN traffic). Stripping only the first two
        bytes of the transfer leaves status bytes in the middle of the data.
        """
        out = bytearray()
        for start in range(0, len(chunk), self._packet_size):
            packet = chunk[start:start + self._packet_size]
            if len(packet) <= 2:
                continue
            if self.line_errors_fatal and packet[1] & 0x8E != 0:
                raise RuntimeError('Error reading data (FTDI line status {:#04x})'.format(packet[1]))
            out += packet[2:]
        if self._read_bitmask is not None:
            for i in range(len(out)):
                out[i] ^= self._read_bitmask
        return out

    def read_available(self, timeout_ms: int = 5) -> bytes:
        """Whatever arrives in one bulk read (already unmasked), plus anything left over by `read`.

        Args:
            timeout_ms (int, optional): How long to wait for data. Defaults to 5.

        Returns:
            bytes: data read, possibly empty
        """
        out = bytearray(self._pending)
        self._pending.clear()
        try:
            out += self._unpack(self.dev.read(self._in_ep, self.READ_SIZE, timeout=timeout_ms))
        except usb.core.USBTimeoutError:
            pass
        return bytes(out)

    def read(self, size: int, retry: bool = True) -> bytes:
        """Read from the Serial interface (responses)

        Bytes beyond `size` are kept for the next call instead of being lost.

        Args:
            size (int): size of data to read

        Returns:
            bytes: data read
        """
        out = bytearray(self._pending[:size])
        del self._pending[:size]
        retrys = 100 if retry else 1

        for i in range(0, retrys):
            if len(out) >= size:
                break
            try:
                body = self._unpack(self.dev.read(self._in_ep, self.READ_SIZE, timeout=500))
            except usb.core.USBTimeoutError:
                continue
            take = size - len(out)
            out += body[:take]
            self._pending += body[take:]

        return out

    def set_latency_timer(self, value: int):
        """Set latency timer"""
        self.dev.ctrl_transfer(self.REQ_OUT, 0x09, value, 0x00, None)

    def set_dtr(self, on: bool):
        """Set DTR"""
        self.dev.ctrl_transfer(
            self.REQ_OUT, 0x01, 0x101 if on else 0x100, 0x00, None)

    def set_rts(self, on: bool):
        """Set RTS"""
        self.dev.ctrl_transfer(
            self.REQ_OUT, 0x01, 0x202 if on else 0x200, 0x00, None)

    def set_line_property(self, databits: int, parity: Parity, stopbits: StopBits, set_break: bool):
        """Set line property"""
        self.dev.ctrl_transfer(
            self.REQ_OUT, 0x04, (databits & 0x0F) | parity | stopbits | ((0x01 << 14) if set_break else 0), 0x00, None)

    def set_baudrate(self, baudrate: int):
        """Set baudrate"""
        clock: int = 3000000
        div8 = int(round((8 * clock) / baudrate))
        div = div8 >> 3
        div |= self.FRAC_DIV_CODE[div8 & 0x7] << 14
        if div == 1:
            div = 0
        elif div == 0x4001:
            div = 1
        value = div & 0xFFFF
        index = (div >> 16) & 0xFFFF

        self.dev.ctrl_transfer(self.REQ_OUT, 0x03, value, index, None)
