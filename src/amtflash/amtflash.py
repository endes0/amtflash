from ftdibus import FTDIBus

class AMTFlasher:

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

        self._purge()
        self._bus.write(b'\x21\x55')
        response = self._bus.read(2)
        for i in range(0, len(response)):
            response[i] ^= 0xFF  # This is equivalent to ~ in reality
            response[i] ^= 0x33
        self._bus.write(bytearray([0x21, 0x56, response[0], response[1]]))
        ok = self._bus.read(1)
        if ok[0] != 0x33:
            raise RuntimeError('Handshake failed, read: ' +
                               str(ok[0]) + ', expected: 0x33')

        self._bus.write_EE(0x5001, bytearray([]))

        self._purge()
        self._bus.write(b'\x26\x00\x01\x00\x00')
        ok = self._bus.read(1)
        if ok[0] != 'U':
            raise RuntimeError(
                'Handshake last phase (checksum) failed, read: ' + str(ok[0]) + ', expected: U')

    def _purge(self):
        readed = self._bus.read(1)
        while len(readed) > 0:
            readed = self._bus.read(1)

    # Public methods

    def get_voltage(self) -> float:
        data = self._bus.read_EE(0x3000, 0x2)
        value = data[1] + (data[0] << 8)
        return value / 52.01
    
    def get_usages(self) -> int:
        return self._bus.read_EE(0x6000, 0x1)[0]
    
    def get_security_num(self) -> bytes:
        data = self._bus.read_EE(0x5000, 0x8)
        result = bytearray(8)
        for i in range(8):
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
        return ok[0] == 'U'
    
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
        return ok[0] == 'U'
    
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
        return ok[0] == 'U'
    
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
        return ok[0] == 'U'
    
    def unknown_0x2a(self) -> int:
        """ WARNING: Untested method
        This command seems to do nothing and always return error(0x55).
        """        
        self._purge()
        self._bus.write(b'\x2a')
        ok = self._bus.read(1)
        return ok[0]
    

class KWPInterface:
    def __init__(self, parent: AMTFlasher) -> None:
        self._parent = parent

    def set_baudrate(self, baudrate: int) -> None:
        self._parent._bus.set_baudrate(baudrate)
    
    def set_line_property(self, databits: int, parity: FTDIBus.Parity, stopbits: FTDIBus.StopBits, set_break: bool):
        self._parent._bus.set_line_property(databits, parity, stopbits, set_break)
    
    def set_dtr(self, state: bool):
        self._parent._bus.set_dtr(state)
    
    def set_rts(self, state: bool):
        self._parent._bus.set_rts(state)

    def send_byte(self, byte: int) -> bool:
        """ Send a byte with the current baudrate.

        Args:
            byte (int): Byte to send
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x25\x04' + byte.to_bytes(1, 'little'))
        ok = self._parent._bus.read(1)
        return ok[0] == 'U'


    def send_byte_custom_baud(self, byte: int, baudrate: int = 5) -> None:
        """ Send a byte with a custom baudrate very slow baudrate. Normally used to send the 5 bauds init sequence.

        Args:
            byte (int): Byte to send
            baudrate (int, optional): Baudrate to use. Defaults to 5.
        """        
        delay = int(1000000 / baudrate)
        self._parent._purge()
        self._parent._bus.write(b'\x25\x03' + delay.to_bytes(1, 'little') + byte.to_bytes(1, 'little'))
    
    def send_bytes(self, data: bytes, delay_between_bytes = 0) -> None:
        """ Send multiple bytes with a delay between each byte.

        Args:
            data (bytes): Bytes to send
            delay_between_bytes (int, optional): delay in ms between each byte. Defaults to 0.
        """        

        # There is also "\x25\x05" with seems to do the same
        self._parent._purge()
        self._parent._bus.write(b'\x25\x02' + len(data).to_bytes(2, 'little') + delay_between_bytes.to_bytes(1, 'little') + data)

    # Untested methods
    def send_fast_init(self, data: bytes, init_pulse_delay = 1, delay_between_bytes = 0) -> None:
        """ WARNING: Untested method
        This command seems to send an initial pulse to the KWP bus before sending the data. Maybe used to do a fast init.

        Args:
            data (bytes): Bytes to send
            init_pulse_delay (int, optional): Duration in ms of the pulse. Defaults to 1.
            delay_between_bytes (int, optional):  delay in ms between each byte. Defaults to 0.
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x25\x01' + len(data).to_bytes(1, 'little') + delay_between_bytes.to_bytes(1, 'little') + init_pulse_delay.to_bytes(1, 'little') + data)
    

class CANInterface:

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
        Mode0 = 0
        Mode1 = 1
        Mode2 = 2
        Raw0 = 3
        Mode4 = 4
        Raw1 = 5
        Mode6 = 6
        Mode7 = 7
        Mode8 = 8


    def __init__(self, parent: AMTFlasher) -> None:
        self._parent = parent
    
    def close(self) -> None:
        self._parent._bus.write(b'\x30\x0A')

    def reset_controller(self) -> bool:
        self._parent._purge()
        self._parent._bus.write(b'\x30\x01')
        ok = self._parent._bus.read(1)
        return ok[0] == 'U'
    
    def enable_controller(self) -> bool:
        self._parent._purge()
        self._parent._bus.write(b'\x30\x09')
        ok = self._parent._bus.read(1)
        return ok[0] == 'U'
    
    def setup(self, acceptance_code: int, acceptance_mask: int, can_identifier: int, rx_filter_can_indentifier: int, bus_timing: BusTimming = BusTimming.Mode7, extended_frame: bool = False, encapsulation: Encapsulation = Encapsulation.Raw0, custom_bus_timing_0: int = 0, custom_bus_timing_1: int = 0) -> bool:
        """ Setup the CAN controller

        Args:
            acceptance_code (int): Acceptance code for filtering the incoming CAN messages (see [SJA1000 datasheet](https://www.nxp.com/docs/en/data-sheet/SJA1000.pdf))
            acceptance_mask (int): Acceptance mask for filtering the incoming CAN messages (see [SJA1000 datasheet](https://www.nxp.com/docs/en/data-sheet/SJA1000.pdf))
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
        final += acceptance_code.to_bytes(4, 'little')
        final += acceptance_mask.to_bytes(4, 'little')
        final += can_identifier.to_bytes(4, 'little')
        final += rx_filter_can_indentifier.to_bytes(4, 'little')
        if extended_frame:
            final += b'\x01'
        else:
            final += b'\x00'
        final += b'\x00' # Normal transmmision
        final += encapsulation.to_bytes(1, 'little')

        self._parent._purge()
        self._parent._bus.write(final)
        ok = self._parent._bus.read(1)
        return ok[0] == 'U'
    
    def send_max_8_bytes(self, msg: bytes):
        """ Send a CAN message of maximum 8 bytes

        Args:
            msg (bytes): The CAN message to send
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x30\x03' + len(msg).to_bytes(1, 'little') + msg)
    
    def send(self, msg: bytes):
        """ Send a CAN message

        Args:
            msg (bytes): The CAN message to send
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x30\x11' + len(msg).to_bytes(2, 'little') + msg)
    

    # Untested

    def receive(self) -> bytes:
        """ Warning: Untested and unfinished method
        Get the last received CAN message

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
        return ok[0] == 'U'
    
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
        return ok[0] == 'U'
    
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
        return ok[0] == 'U'
        
    def change_rx_filter_can_identifier(self, can_identifier: int) -> bool:
        """ Warning: Untested method
        Change the RX filter CAN identifier

        Args:
            can_identifier (int): The CAN identifier

        Returns:
            bool: True if the RX filter CAN identifier was set successfully
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x30\x0b' + can_identifier.to_bytes(4, 'little'))
        ok = self._parent._bus.read(1)
        return ok[0] == 'U'

    def change_can_identifer(self, can_identifier: int) -> bool:
        """ Warning: Untested method
        Change the CAN identifier

        Args:
            can_identifier (int): The CAN identifier

        Returns:
            bool: True if the CAN identifier was set successfully
        """        
        self._parent._purge()
        self._parent._bus.write(b'\x30\x0c' + can_identifier.to_bytes(4, 'little'))
        ok = self._parent._bus.read(1)
        return ok[0] == 'U'
    
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
        return ok[0] == 'U'

