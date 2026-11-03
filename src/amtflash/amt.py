from amtflash.ftdibus import FTDIBus

class AMTFlash:

    def __init__(self, _custom_vid=None, _custom_pid=None) -> None:
        if _custom_vid is not None:
            self._bus = FTDIBus(_custom_vid, _custom_pid)
        else:
            self._bus = FTDIBus()
        self._bus.open()
        self._hanshake()
        self.kwp = KWPInterface(self)
        self.can = CANInterface(self)

    def _hanshake(self):
        magic_num = self._bus.read_EE(0x1000, 2)
        if magic_num[0] != 0x33:
            raise RuntimeError('Invalid magic number, read: ' +
                               str(magic_num[0]) + ', expected: 0x33')

        bitmasks = self._bus.read_EE(0x2000, 2)
        self._bus._write_bitmask = bitmasks[0]
        self._bus._read_bitmask = bitmasks[1]

        # A previous session that opened the CAN leaves the device in that mode, which survives the USB reset and
        # the buffer purges: the challenge then answers 0xAA. Closing the CAN first makes opening repeatable.
        self._purge()
        self._bus.write(b'\x30\x0A')
        self._purge()
        self._bus.write(b'\x21\x55')
        response = self._bus.read(2)
        for i in range(0, len(response)):
            response[i] ^= 0xFF  # This is equivalent to ~ in reality
            response[i] ^= 0x33
        self._purge()
        self._bus.write(bytearray([0x21, 0x56, response[0], response[1]]))
        ok = self._bus.read(1)
        if ok[0] != 0x33:
            raise RuntimeError('Handshake failed, read: ' +
                               str(ok[0]) + ', expected: 0x33')

        self._bus.write_EE(0x5001, bytearray([]))

        self._purge()
        self._bus.write(b'\x26\x00\x01\x00\x00')
        ok = self._bus.read(1)
        if ok[0] != ord('U'):
            raise RuntimeError(
                'Handshake last phase (checksum) failed, read: ' + str(ok[0]) + ', expected: U')

    def _purge(self):
        readed = self._bus.read_available(50)
        while len(readed) > 0:
            readed = self._bus.read_available(50)

    # Public methods

    def resync(self):
        """Reset the FTDI and do the handshake again.

        After a K-line command the device stops answering commands until it is reopened, so this is the only
        reliable way back to the command state (e.g. to open the CAN again).
        """
        self._bus.reset()
        self._bus.line_errors_fatal = True
        self._hanshake()

    def get_voltage(self) -> float:
        data = self._bus.read_EE(0x3000, 0x2)
        value = data[1] + (data[0] << 8)
        return value / 52.01
    
    def get_usages(self) -> int:
        return self._bus.read_EE(0x6000, 0x2)[0]
    
    def get_security_num(self) -> bytes:
        data = self._bus.read_EE(0x5000, 0x8)
        result = bytearray(len(data))
        for i in range(0, len(data)):
            result[i] = data[i] ^ self._bus._write_bitmask
        return result
    
    def get_cert(self) -> bytes:
        return self._bus.read_EE(0x4000, 0x200)
    
    def get_version(self) -> int:
        self._purge()
        self._bus.write(b'\x31')
        data = self._bus.read(2)
        return data[1] + (data[0] << 8)
    
    def get_version_str(self) -> str:
        self._purge()
        self._bus.write(b'\x22')
        str_len = self._bus.read(1)[0]
        data = self._bus.read(str_len)
        return data.decode('utf-8')
    
    def set_delay(self, delay: int) -> bool:
        self._purge()
        self._bus.write(b'\x24' + delay.to_bytes(1, 'big'))
        ok = self._bus.read(1)
        return ok[0] == ord('U')
    
    # Untested methods

    def disable_flash_write(self) -> bool:
        """ WARNING: Untested method
        This command seems to disable the writes/erases to the flash memory of the microcontroller.

        Returns:
            bool: True if the command was successful, False otherwise
        """        
        self._purge()
        self._bus.write(b'\x20')
        ok = self._bus.read(1)
        return ok[0] == ord('U')
    
    def set_pin_0(self, high: bool) -> bool:
        """ WARNING: Untested method
        This command sets the pin 0 (of the port 1) of the microcontroller to high or low.

        Args:
            high (bool): True to set the pin to high, False to set it to low

        Returns:
            bool: True if the command was successful, False otherwise
        """        
        self._purge()
        self._bus.write(b'\x27\x00' + (b'\x01' if high else b'\x00'))
        ok = self._bus.read(1)
        return ok[0] == ord('U')
    
    def set_pin_2(self, high: bool) -> bool:
        """ WARNING: Untested method
        This command sets the pin 2 (of the port 1) of the microcontroller to high or low.

        Args:
            high (bool): True to set the pin to high, False to set it to low

        Returns:
            bool: True if the command was successful, False otherwise
        """        
        self._purge()
        self._bus.write(b'\x27\x01' + (b'\x01' if high else b'\x00'))
        ok = self._bus.read(1)
        return ok[0] == ord('U')
    
    def unknown_0x2a(self) -> int:
        """ WARNING: Untested method
        This command seems to do nothing and always return error(0x55).
        """        
        self._purge()
        self._bus.write(b'\x2a')
        ok = self._bus.read(1)
        return ok[0]
    

class KWPInterface:
    """ K-line, validated on a vehicle (5 baud init, then byte by byte at 10400 bps).

    Hardware notes: the K-line is driven by UART1 of the C8051F340 (TX on P0.4, RX on P0.5). P0.6 goes high
    while a byte is being sent and cuts the receiver, so **there is no echo** of transmitted bytes. Bytes from the
    ECU arrive in the normal bulk stream, at the baudrate set with `set_baudrate` (the FTDI SET_BAUDRATE request).
    The K-line pull-up needs 12 V on the OBD connector.

    After any K-line command the device stops answering regular commands: call `AMTFlash.resync()` before
    using the CAN again.
    """

    # Timer tick used by `send_byte_custom_baud`: about 1.006 ms per tick.
    TICK_US = 1006

    # Bit time of the ISO 9141 / KWP2000 5 baud init: 199 ticks = 200.2 ms.
    FIVE_BAUD_TICKS = 199

    def __init__(self, parent: AMTFlash) -> None:
        self._parent = parent

    def set_baudrate(self, baudrate: int) -> None:
        self._parent._bus.set_baudrate(baudrate)

    def receive(self, timeout_ms: int = 100, quiet_ms: int = 20) -> bytes:
        """ Bytes received on the K-line.

        Args:
            timeout_ms (int, optional): Give up if nothing arrives within this time. Defaults to 100.
            quiet_ms (int, optional): Stop after this long without new bytes, once something arrived. Defaults to 20.

        Returns:
            bytes: received bytes (no echo of our own bytes, see the class notes)
        """
        import time
        self._parent._bus.line_errors_fatal = False
        out = bytearray()
        start = last = time.monotonic()
        while True:
            now = time.monotonic()
            if (now - start) * 1000 >= timeout_ms or (out and (now - last) * 1000 >= quiet_ms):
                break
            data = self._parent._bus.read_available(5)
            if data:
                out += data
                last = time.monotonic()
        return bytes(out)

    def five_baud_init(self, address: int) -> None:
        """ ISO 9141 / KWP2000 5 baud init of `address`.

        Same as `send_byte_custom_baud(address, 5)`. Set the K-line baudrate (e.g. 10400) and leave the line idle
        for ~300 ms before calling it. The device does not answer this command and is busy for the 10 bits
        (~2 s); the sync byte 0x55 and the two key bytes from the ECU then arrive with `receive`.

        Args:
            address (int): ECU address (e.g. 0x33 for OBD, 0x04 for the ABS of some Mitsubishi vehicles)
        """
        self._parent._bus.line_errors_fatal = False
        self._parent._purge()
        self._parent._bus.write(b'\x25\x03' + self.FIVE_BAUD_TICKS.to_bytes(1, 'big') + address.to_bytes(1, 'big'))
    
    def set_line_property(self, databits: int, parity: FTDIBus.Parity, stopbits: FTDIBus.StopBits, set_break: bool):
        self._parent._bus.set_line_property(databits, parity, stopbits, set_break)
    
    def set_dtr(self, state: bool):
        self._parent._bus.set_dtr(state)
    
    def set_rts(self, state: bool):
        self._parent._bus.set_rts(state)

    def send_byte(self, byte: int) -> bool:
        """ Send a byte on the K-line with the current baudrate.

        The device answers 'U' (0x55) after the byte has left. The ECU answer follows in the same stream, so do
        not purge before calling `receive`. Remember that 0x55 is also the ISO 9141 sync byte.

        Args:
            byte (int): Byte to send

        Returns:
            bool: True if the device confirmed the byte
        """
        self._parent._bus.line_errors_fatal = False
        self._parent._purge()
        self._parent._bus.write(b'\x25\x04' + byte.to_bytes(1, 'big'))
        ok = self._parent._bus.read(1)
        return len(ok) == 1 and ok[0] == ord('U')

    def send_byte_custom_baud(self, byte: int, baudrate: int = 5) -> None:
        """ Send a byte with a custom, very slow baudrate. Normally used to send the 5 bauds init sequence.

        The device turns UART1 off and bit-bangs the byte on the TX pin (start bit, 8 data bits LSB first, stop
        bit), holding each bit for a number of ~1.006 ms timer ticks given in **one byte**. It sends no answer.
        A malformed command (e.g. a 2 byte delay) leaves the device stuck until the USB is reopened.

        Args:
            byte (int): Byte to send
            baudrate (int, optional): Baudrate to use, between 4 and 990. Defaults to 5.
        """
        ticks = int(round(1000000 / baudrate / self.TICK_US))
        if not 1 <= ticks <= 255:
            raise ValueError('Baudrate out of range for a one byte tick count: ' + str(baudrate))
        self._parent._bus.line_errors_fatal = False
        self._parent._purge()
        self._parent._bus.write(b'\x25\x03' + ticks.to_bytes(1, 'big') + byte.to_bytes(1, 'big'))

    # Untested methods

    def send_bytes(self, data: bytes, delay_between_bytes = 0) -> None:
        """ WARNING: Not validated on a vehicle
        Send multiple bytes through UART1.

        The length is **big endian**: with a little endian length the device waits for hundreds of bytes and
        swallows the following commands. Whether the byte after the length really is a delay was not confirmed;
        the firmware also uses the general delay set with `AMTFlash.set_delay`. `send_byte` is the validated path.

        Args:
            data (bytes): Bytes to send
            delay_between_bytes (int, optional): delay in ms between each byte. Defaults to 0.
        """

        # There is also "\x25\x05" with seems to do the same
        self._parent._bus.line_errors_fatal = False
        self._parent._purge()
        self._parent._bus.write(b'\x25\x02' + len(data).to_bytes(2, 'big') + delay_between_bytes.to_bytes(1, 'big') + data)

    def send_fast_init(self, data: bytes, init_pulse_ticks = 25) -> None:
        """ WARNING: Not validated on a vehicle
        Fast init: the TX pin goes low and then high for `init_pulse_ticks` timer ticks each (~1.006 ms per tick;
        25 ticks gives the 25 ms + 25 ms of ISO 14230), then the `data` bytes and a checksum computed by the
        device are sent through UART1.

        Args:
            data (bytes): Bytes to send after the pulse (e.g. the StartCommunication request without checksum)
            init_pulse_ticks (int, optional): Duration of the low and of the high phase, in ticks. Defaults to 25.
        """
        self._parent._bus.line_errors_fatal = False
        self._parent._purge()
        self._parent._bus.write(b'\x25\x01' + len(data).to_bytes(1, 'big') + init_pulse_ticks.to_bytes(1, 'big') + data)
    

class CANInterface:
    """ CAN through the SJA1000 of the cable (16 MHz clock), validated on a vehicle at 500 kbps.

    All 32 bit fields of the commands are **big endian**. Received frames come unsolicited in the bulk stream
    (encapsulation mode 0), interleaved with the 'U' answers to commands: read them with `receive`.

    WARNING: the device buffer overflows on a busy bus. On a vehicle with ~820 frames/s (500 kbps, car moving)
    about 0.1% of the frames come with fewer data bytes than announced, and the next frame's header ends up
    inside the data. The stream has no checksum, so corruption that keeps the length cannot be detected at all:
    it shows up as wrong values in any signal. Keep the received traffic low (an estimated ~500 frames/s or
    less; the exact limit was not measured) with the acceptance filter (`standard_id_filter`) instead of
    receiving everything.
    """

    # SJA1000 BTR0/BTR1 at 16 MHz for BusTimming.Custom (datasheet table).
    BTR = {
        1000000: (0x00, 0x14),
        500000: (0x00, 0x1C),
        250000: (0x01, 0x1C),
        125000: (0x03, 0x1C),
        100000: (0x04, 0x1C),
        50000: (0x09, 0x1C),
    }

    # Mask bits set to 1 are "don't care": this accepts every frame.
    ACCEPT_ALL = (0x00000000, 0xFFFFFFFF)

    class BusTimming(int):
        Custom = 0
        Mode1 = 1
        Mode2 = 2
        Mode3 = 3
        Mode4 = 4
        Mode5 = 5
        Mode6 = 6
        Mode7 = 7 #50000 bauds
        Mode8 = 8
        Mode9 = 9
    
    class Encapsulation(int):
        #Maybe related to ISO 15765?
        # Mode0: every received frame comes alone in the stream as `55 | extended (00/01) | id (2 or 4 bytes) |
        # length (2) | data`, big endian. The other modes deliver pieces of the frame.
        Mode0 = 0
        Stream = 0
        Mode1 = 1
        Mode2 = 2
        Raw0 = 3
        Mode4 = 4
        Raw1 = 5
        Mode6 = 6
        Mode7 = 7
        Mode8 = 8


    def __init__(self, parent: AMTFlash) -> None:
        self._parent = parent
        self._rx = bytearray()
        self._frames = []
        # Frames dropped because the next one did not start where their length said (see the class warning).
        self.truncated = 0

    @staticmethod
    def standard_id_filter(identifier: int, dont_care: int = 0) -> tuple:
        """ Acceptance code and mask for 11 bit identifiers.

        The firmware copies the code and the mask of `setup` straight into ACR0..3 and AMR0..3 and never enables
        the SJA1000 dual filter mode, so there is **one rule only**: a frame passes when its identifier matches
        `identifier` on every bit that is 0 in `dont_care`. In single filter mode the 11 bit identifier sits in
        bits 31..21; RTR and the data bytes are left as "don't care".

        Examples:
            standard_id_filter(0x7E8): only 0x7E8
            standard_id_filter(0x7E8, 0x007): 0x7E8..0x7EF (OBD responses)
            standard_id_filter(0x700, 0x0FF): 0x700..0x7FF (diagnostic responses of most makers)
            standard_id_filter(0x100, 0x6FF): bit 8 set: 0x1xx, 0x3xx, 0x5xx and 0x7xx

        Args:
            identifier (int): identifier to match
            dont_care (int, optional): identifier bits to ignore. Defaults to 0 (exact match).

        Returns:
            tuple: (acceptance_code, acceptance_mask) for `setup`
        """
        code = (identifier & 0x7FF) << 21
        mask = ((dont_care & 0x7FF) << 21) | 0x1FFFFF
        return code, mask

    def close(self) -> None:
        self._parent._bus.write(b'\x30\x0A')

    def reset_controller(self) -> bool:
        self._parent._purge()
        self._parent._bus.write(b'\x30\x01')
        ok = self._parent._bus.read(1)
        return ok[0] == ord('U')
    
    def enable_controller(self) -> bool:
        """ Enable the controller and zero the two transmit delays.

        With the default delays (`30 12` of the CAN and the general `24`) the device accepts about one send every
        250 ms and silently drops the commands in between. After a power cycle the general delay is the one that
        holds. Even with both at zero the device has no queue: keep ~1 ms between commands.
        """
        self._parent._purge()
        self._parent._bus.write(b'\x30\x09')
        ok = self._parent._bus.read(1)
        if len(ok) != 1 or ok[0] != ord('U'):
            return False
        return self.set_transmission_delay(0) and self._parent.set_delay(0)

    def setup_bitrate(self, bitrate: int, acceptance_code: int = 0x00000000, acceptance_mask: int = 0xFFFFFFFF, can_identifier: int = 0x7DF, extended_frame: bool = False) -> bool:
        """ `setup` with explicit SJA1000 timing for `bitrate`, receiving in stream mode.

        Args:
            bitrate (int): one of `CANInterface.BTR`
            acceptance_code (int, optional): see `standard_id_filter`. Defaults to accept all.
            acceptance_mask (int, optional): see `standard_id_filter`. Defaults to accept all.
            can_identifier (int, optional): identifier to transmit with. Defaults to 0x7DF.
            extended_frame (bool, optional): Defaults to False.
        """
        btr0, btr1 = self.BTR[bitrate]
        return self.setup(acceptance_code, acceptance_mask, can_identifier, 0, CANInterface.BusTimming.Custom,
                          extended_frame, CANInterface.Encapsulation.Stream, btr0, btr1)

    def setup(self, acceptance_code: int, acceptance_mask: int, can_identifier: int, rx_filter_can_indentifier: int, bus_timing: BusTimming = BusTimming.Mode7, extended_frame: bool = False, encapsulation: Encapsulation = Encapsulation.Stream, custom_bus_timing_0: int = 0, custom_bus_timing_1: int = 0) -> bool:
        """ Setup the CAN controller

        Args:
            acceptance_code (int): Acceptance code for filtering the incoming CAN messages (see `standard_id_filter` and the [SJA1000 datasheet](https://www.nxp.com/docs/en/data-sheet/SJA1000.pdf)). Use `CANInterface.ACCEPT_ALL` to receive everything.
            acceptance_mask (int): Acceptance mask for filtering the incoming CAN messages. Bits set to 1 are "don't care"; a mask of 0 only accepts the identifier equal to the code.
            can_identifier (int): Identifer to transmit the CAN messages with
            rx_filter_can_indentifier (int): The exact identifier of the CAN messages to receive. Set to 0 to receive all messages.
            bus_timing (BusTimming, optional): The different timmings modes. Defaults to BusTimming.Mode7.
            extended_frame (bool, optional): Transmit and receive extended frames. Defaults to False.
            encapsulation (Encapsulation, optional): Those diferents modes encapsulates the message inside another type of packets, Except for Raws modes. Defaults to Encapsulation.Raw0.
            custom_bus_timing_0 (int, optional): If BusTimming.Custom is used, this is the value set for the Bus Timing Register 0 (BTR0) (see [SJA1000 datasheet](https://www.nxp.com/docs/en/data-sheet/SJA1000.pdf)). Defaults to 0.
            custom_bus_timing_1 (int, optional): If BusTimming.Custom is used, this is the value set for the Bus Timing Register 1 (BTR1) (see [SJA1000 datasheet](https://www.nxp.com/docs/en/data-sheet/SJA1000.pdf)). Defaults to 0.

        Returns:
            bool: True if the setup was successful
        """        
        final = b'\x30\x10'
        final += bus_timing.to_bytes(1, 'little')
        if bus_timing == CANInterface.BusTimming.Custom:
            final += custom_bus_timing_0.to_bytes(1, 'little')
            final += custom_bus_timing_1.to_bytes(1, 'little')
        # Big endian: with little endian the frames left with identifier 0 on the bus.
        final += acceptance_code.to_bytes(4, 'big')
        final += acceptance_mask.to_bytes(4, 'big')
        final += can_identifier.to_bytes(4, 'big')
        final += rx_filter_can_indentifier.to_bytes(4, 'big')
        if extended_frame:
            final += b'\x01'
        else:
            final += b'\x00'
        final += b'\x00' # Normal transmmision
        final += encapsulation.to_bytes(1, 'little')

        self._parent._purge()
        self._rx.clear()
        self._frames.clear()
        self._parent._bus.write(final)
        ok = self._parent._bus.read(1)
        return len(ok) == 1 and ok[0] == ord('U')

    def send_max_8_bytes(self, msg: bytes):
        """ Send a CAN message of maximum 8 bytes, with the identifier of `setup` or `change_can_identifer`

        No purge here: it would throw away the received frames waiting in the stream. The device has no
        transmit queue: back to back sends get lost, ~1 ms between commands is enough.

        Args:
            msg (bytes): The CAN message to send
        """
        self._parent._bus.write(b'\x30\x03' + len(msg).to_bytes(1, 'big') + msg)

    def send(self, msg: bytes):
        """ WARNING: Do not use with a little endian length
        Send a long CAN message (the firmware splits it, in a way that looks like ISO-TP but does not follow it).

        With the little endian length of the original code the device dumped ~730 frames on the bus. Split the
        message on the host (ISO-TP) and use `send_max_8_bytes` instead.

        Args:
            msg (bytes): The CAN message to send
        """
        self._parent._purge()
        self._parent._bus.write(b'\x30\x11' + len(msg).to_bytes(2, 'big') + msg)

    def receive(self, timeout_ms: int = 5):
        """ Next received frame from the stream (encapsulation mode 0), or None.

        A frame is only accepted when the byte after it starts another frame or is a command answer
        (0x55 / 0xAA): the device sometimes announces 8 data bytes and sends fewer, and without this check the
        next frame's header would be returned as data. Rejected frames are counted in `truncated`.

        Args:
            timeout_ms (int, optional): How long to wait for data. Defaults to 5.

        Returns:
            tuple: (identifier, data, extended) or None
        """
        if not self._frames:
            self._poll(timeout_ms)
        return self._frames.pop(0) if self._frames else None

    def _poll(self, timeout_ms: int) -> list:
        """ Read the stream once, queue the complete frames and return the command answers (True = 'U')."""
        data = self._parent._bus.read_available(timeout_ms)
        self._rx += data
        return self._parse(quiet=not data)

    def _parse(self, quiet: bool) -> list:
        answers = []
        rx = self._rx
        pos = 0
        while pos < len(rx):
            first = rx[pos]
            if first == 0xAA:  # refused command
                answers.append(False)
                pos += 1
                continue
            if first != 0x55:
                pos += 1
                continue
            if len(rx) - pos < 2:
                if quiet:  # a lone 'U' at the end, with the link quiet
                    answers.append(True)
                    pos += 1
                break
            if rx[pos + 1] > 1:  # 'U' not followed by 00/01: answer to a command
                answers.append(True)
                pos += 1
                continue
            id_end = pos + 2 + (4 if rx[pos + 1] == 1 else 2)
            if len(rx) < id_end + 2:
                break
            size = int.from_bytes(rx[id_end:id_end + 2], 'big')
            if size > 8:  # impossible header: out of sync
                pos += 1
                continue
            end = id_end + 2 + size
            # The last frame of the buffer waits for the next byte, or for the link to go quiet.
            if len(rx) < end or (len(rx) == end and not quiet):
                break
            if len(rx) > end and rx[end] not in (0x55, 0xAA):
                self.truncated += 1
                pos += 1
                continue
            self._frames.append((int.from_bytes(rx[pos + 2:id_end], 'big'), bytes(rx[id_end + 2:end]), rx[pos + 1] == 1))
            pos = end
        del rx[:pos]
        return answers

    # Untested

    def receive_last(self) -> bytes:
        """ Warning: Untested and unfinished method
        Get the last received CAN message. In encapsulation mode 0 it only answers 0xAA.

        Returns:
            bytes: The last received CAN message
        """
        self._parent._purge()
        self._parent._bus.write(b'\x30\x08')
        return self._parent._bus.read(8)
    
    def get_CAN_status(self) -> int:
        """ Warning: Untested method
        Get the status flag of the CAN controller

        Returns:
            int: The status flag of the CAN controller
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x30\x05')
        return self._parent._bus.read(1)[0]

    def get_error_code(self) -> int:
        """ Warning: Untested method
        Get the error code of the CAN controller

        Returns:
            int: The error code of the CAN controller
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x30\x06')
        return self._parent._bus.read(1)[0]
    
    def set_lisent_mode_on(self) -> bool:
        """ Warning: Untested method
        Set the CAN controller in listen only mode

        Returns:
            bool: True if the listen only mode was set successfully
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x30\x07')
        ok = self._parent._bus.read(1)
        return ok[0] == ord('U')
    
    def set_transmission_delay(self, delay: int) -> bool:
        """ Warning: Untested method
        Set the delay between each byte set to the CAN Controller

        Args:
            delay (int): Delay in ms

        Returns:
            bool: True if the delay was set successfully
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x30\x12' + delay.to_bytes(1, 'little'))
        ok = self._parent._bus.read(1)
        return ok[0] == ord('U')
    
    def change_bus_timing(self, bus_timming: BusTimming, custom_bus_timing_0: int = 0, custom_bus_timing_1: int = 0) -> bool:
        """ Warning: Untested method
        Change the bus timing

        Args:
            bus_timming (BusTimming): The different timmings modes
            custom_bus_timing_0 (int, optional): If BusTimming.Custom is used, this is the value set for the Bus Timing Register 0 (BTR0) (see [SJA1000 datasheet](https://www.nxp.com/docs/en/data-sheet/SJA1000.pdf)). Defaults to 0.
            custom_bus_timing_1 (int, optional): If BusTimming.Custom is used, this is the value set for the Bus Timing Register 1 (BTR1) (see [SJA1000 datasheet](https://www.nxp.com/docs/en/data-sheet/SJA1000.pdf)). Defaults to 0.

        Returns:
            bool: True if the bus timing was set successfully
        """        
        final = b'\x30\x13'
        final += bus_timming.to_bytes(1, 'little')
        if bus_timming == CANInterface.BusTimming.Custom:
            final += custom_bus_timing_0.to_bytes(1, 'little')
            final += custom_bus_timing_1.to_bytes(1, 'little')
        
        self._parent._purge()
        self._parent._bus.write(final)
        ok = self._parent._bus.read(1)
        return ok[0] == ord('U')
        
    def change_rx_filter_can_identifier(self, can_identifier: int) -> bool:
        """ Warning: Untested method
        Change the RX filter CAN identifier

        Args:
            can_identifier (int): The CAN identifier

        Returns:
            bool: True if the RX filter CAN identifier was set successfully
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x30\x0b' + can_identifier.to_bytes(4, 'big'))
        ok = self._parent._bus.read(1)
        return ok[0] == ord('U')

    def change_can_identifer(self, can_identifier: int) -> bool:
        """ Change the identifier used to transmit, without reconfiguring the controller (validated, big endian).

        The answer may come after received frames: frames read here are kept for `receive`.

        Args:
            can_identifier (int): The CAN identifier

        Returns:
            bool: True if the CAN identifier was set successfully
        """
        self._parent._bus.write(b'\x30\x0c' + can_identifier.to_bytes(4, 'big'))
        import time
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            answers = self._poll(5)
            if answers:
                return answers[0]
        return False
    
    def change_frame_format(self, extended_frame: bool) -> bool:
        """ Warning: Untested method
        Change the frame format

        Args:
            extended_frame (bool): True for extended frame, False for standard frame

        Returns:
            bool: True if the frame format was set successfully
        """        
        self._parent._purge()
        if extended_frame:
            self._parent._bus.write(b'\x30\x0f\x01')
        else:
            self._parent._bus.write(b'\x30\x0f\x00')
        ok = self._parent._bus.read(1)
        return ok[0] == ord('U')

