import queue
import usb.core
from usb.util import build_request_type, CTRL_OUT, CTRL_IN, CTRL_TYPE_VENDOR, CTRL_RECIPIENT_DEVICE

class FTDIBus:
    """The low level FTDI bus interface to communicate with the AMT flasher
    """

    REQ_OUT = build_request_type(
        CTRL_OUT, CTRL_TYPE_VENDOR, CTRL_RECIPIENT_DEVICE)
    REQ_IN = build_request_type(
        CTRL_IN, CTRL_TYPE_VENDOR, CTRL_RECIPIENT_DEVICE)
    FRAC_DIV_CODE = (0, 3, 2, 4, 1, 5, 6, 7)

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
        self._in_ep = None
        self._out_ep = None
        self._write_bitmask = None
        self._read_bitmask = None
        self._read_buffer = queue.Queue()
        self.dev = None

    def open(self):
        """Open device"""
        self.dev: usb.core.Device = usb.core.find(
            idVendor=self._vid, idProduct=self._pid)
        if self.dev is None:
            raise RuntimeError('Device not found')

        self.dev.reset()
        self.dev.set_configuration()

        cfg: usb.core.Configuration = self.dev.get_active_configuration()
        self._in_ep = cfg[(0, 0)][1].bEndpointAddress
        self._out_ep = cfg[(0, 0)][0].bEndpointAddress

        self.reset()
        self.set_latency_timer(1)

    def close(self):
        """Close device"""
        usb.util.dispose_resources(self.dev)

    def reset(self):
        """Purge device"""
        self.dev.ctrl_transfer(self.REQ_OUT, 0x00, 0x00, 0x00, None)

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
        self.dev.write(self._in_ep, data)

    def read(self, size: int, retry: bool = True) -> bytes:
        """Read from the Serial interface (responses)

        Args:
            size (int): size of data to read

        Returns:
            bytes: data read
        """
        result_buf = bytearray()
        pos = 0
        retrys = 100

        while pos < size and not self._read_buffer.empty():
            result_buf.append(self._read_buffer.get())
            pos += 1

        if pos < size:
            for i in range(0, retrys):
                tempbuf = self.dev.read(self._out_ep, (4 << 10))

                if tempbuf[1] & 0x8E != 0:
                    raise RuntimeError('Error reading data')

                if len(tempbuf) > 2:
                    extract = (size - pos) if (size - pos) <= len(tempbuf)-2 else len(tempbuf)-2
                    result_buf[pos:] = tempbuf[2: extract+2]
                    pos += extract

                    if len(tempbuf)-2 > size:
                        # Put extra data in buffer
                        for i in range(size+2, len(tempbuf)):
                            self._read_buffer.put(tempbuf[i])

                if pos >= size:
                    break

                if not retry:
                    break

        # Apply read bitmask
        if self._read_bitmask is not None:
            for i in range(len(result_buf)):
                result_buf[i] ^= self._read_bitmask

        return result_buf

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
