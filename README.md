# amtflash
Device driver for the AMT Flash (MPPS) v13 OBD Cable

# Usage
``` python
from amtflash import AMTFlash
device = AMTFlash()
print("Version: {}".format(device.get_version()))
print("Version: {}".format(device.get_version_str()))
```

On Windows the device needs the WinUSB driver (e.g. with Zadig); with the original driver libusb cannot claim it.

## K-line (5 baud init)

Validated on a vehicle (Mitsubishi ABS, address 0x04, 10400 bps). The K-line needs 12 V on the OBD connector.
There is no echo of the bytes we send, and after any K-line command the device only answers again after
`resync()`.

``` python
import time
from amtflash import AMTFlash

device = AMTFlash()
kwp = device.kwp

kwp.set_baudrate(10400)
time.sleep(0.3)                      # line idle before the init
kwp.five_baud_init(0x04)             # same as send_byte_custom_baud(0x04, 5): no answer, ~2 s busy
init = kwp.receive(timeout_ms=3500, quiet_ms=300)
sync = init.rfind(0x55)              # 0x55 sync, then the two key bytes
kb1, kb2 = init[sync + 1], init[sync + 2]

time.sleep(0.03)                     # W4
kwp.send_byte(~kb2 & 0xFF)           # key byte 2 inverted; the device answers 'U' after the byte leaves
print(kwp.receive(timeout_ms=300).hex(' '))

device.resync()                      # back to the command state (needed before opening the CAN)
```

`send_byte` sends one byte at the current baudrate; send a request byte by byte. `send_bytes` and
`send_fast_init` follow the command layout but were not validated on a vehicle yet.

## CAN

All 32 bit fields go **big endian**. Received frames arrive unsolicited in the stream (`Encapsulation.Stream`),
read them with `receive()`.

``` python
from amtflash import AMTFlash
from amtflash.amt import CANInterface

device = AMTFlash()
can = device.can
can.reset_controller()

# Receive everything (see the warning below before doing this on a vehicle)
code, mask = CANInterface.ACCEPT_ALL
can.setup_bitrate(500000, code, mask, can_identifier=0x7DF)
can.enable_controller()              # also zeroes the transmit delays

can.send_max_8_bytes(bytes([0x02, 0x01, 0x0C, 0, 0, 0, 0, 0]))   # OBD: engine RPM
frame = can.receive()                # (identifier, data, extended) or None
```

### Acceptance filter

The firmware writes the code and the mask straight into the SJA1000 (ACR0..3, AMR0..3) in single filter mode;
dual filter mode is never enabled, so there is **one rule only**. Mask bits set to 1 are "don't care".
`standard_id_filter(identifier, dont_care)` builds the rule for 11 bit identifiers:

``` python
# Only the engine OBD response
code, mask = CANInterface.standard_id_filter(0x7E8)
# All OBD responses, 0x7E8..0x7EF
code, mask = CANInterface.standard_id_filter(0x7E8, 0x007)
# Every diagnostic response in 0x700..0x7FF
code, mask = CANInterface.standard_id_filter(0x700, 0x0FF)
# Bit 8 set: 0x1xx, 0x3xx, 0x5xx and 0x7xx (some broadcast plus all 0x7xx responses)
code, mask = CANInterface.standard_id_filter(0x100, 0x6FF)

can.setup_bitrate(500000, code, mask, can_identifier=0x7E0)
can.enable_controller()
```

Since there is a single rule, choose it by the traffic it lets through: one mask covering identifiers that are
far apart opens most of the bus.

### Warning: the device buffer fills up on a busy bus

On a vehicle bus with ~820 frames/s (500 kbps, car moving) the device corrupts about 0.1% of the received
frames: it announces 8 data bytes, sends fewer, and the next frame's header lands inside the data. `receive()`
drops those (counted in `can.truncated`), but the stream has no checksum, so corruption that keeps the length
cannot be detected and shows up as wrong values in any signal. Keep the received traffic below about
**500 frames/s** with the acceptance filter. That limit is an estimate, not a measurement.
The USB side is not the bottleneck: reading continuously on the host did not change the rate.

### Do not use

- `send()` (`30 11`) with a little endian length: the device dumped ~730 frames on the bus. Do ISO-TP on the
  host with `send_max_8_bytes`.
- `25 02` with a little endian length, or a malformed `25 03`: the device waits for bytes that never come and
  only recovers after the USB is reopened.
