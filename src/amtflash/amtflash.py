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
    def __init__(self, parent: AMTFlasher) -> None:
        self._parent = parent
    
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
    
    #def setup( 

