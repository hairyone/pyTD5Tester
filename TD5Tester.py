from pyftdi.ftdi import Ftdi
import pyftdi.serialext
import time

READ_BUFFER_SIZE = 127
USE_BREAK = False # break not working
USE_SERIAL = True

# RS232 vs TTL
# https://www.sparkfun.com/tutorials/215

# This method of serial communication is sometimes referred to as TTL serial
# (transistor-transistor logic). Serial communication at a TTL level will
# always remain between the limits of 0V and Vcc, which is often 5V or 3.3V.
# A logic high ('1') is represented by Vcc, while a logic low ('0') is 0V.

# The two differ solely at a hardware level.

# By the RS-232 standard a logic high ('1') is represented by a negative
# voltage – anywhere from -3 to -25V – while a logic low ('0') transmits
# a positive voltage that can be anywhere from +3 to +25V. On most PCs
# these signals swing from -13 to +13V.


################################################################################
def calculate_key(seed):
################################################################################
    count = ((seed >> 0xC & 0x8) + (seed >> 0x5 & 0x4) + (seed >> 0x3 & 0x2) + (seed & 0x1)) + 1

    for idx in range(0, count):
        tap = ((seed >> 1) + (seed >> 2 ) + (seed >> 8 ) + (seed >> 9)) & 1
        tmp = (seed >> 1) | ( tap << 0xF)
        
        if (seed >> 0x3 & 1) and (seed >> 0xD & 1):
            seed = tmp & ~1
        else:
            seed = tmp | 1

    return (seed >> 8, seed & 255)

################################################################################
def set_checksum(data):
################################################################################
    data_len = len(data) - 1
    crc = 0
    for i in range(0, data_len):
        crc = crc + data[i]

    # get leftmost byte, two ways
    # crc = crc & 0xF
    crc = crc % 256

    data[data_len ] = crc % 256  

################################################################################
def logit(data, is_tx):
################################################################################
    print("{} {}".format(
        ">>" if is_tx else "<<",
        ''.join('{:02X} '.format(x) for x in data).rstrip()
    ))

################################################################################
def fast_init():
################################################################################
    HI = bytearray([0x01])
    LO = bytearray([0x00])

    uart = Ftdi()
    uart.open(0x403, 0x6001)
    uart.set_baudrate(10400)
    uart.set_line_property(8, 1, 'N')
    uart.set_bitmode(0x01, 0x01)

    if USE_BREAK:
        uart.set_break(False)
    else:
        uart.write_data(HI)

    # Short sleeps in the loop seem to improve accuracy...
    # Using time.sleep(.025) for the full delay is very jittery.
    # time.perf_counter() can be used instead time.monotonic()
    # requires Python 3.3 or later

    start = time.monotonic()
    while (time.monotonic() <= start + 0.50):
        time.sleep(0.01)

    if USE_BREAK:
        uart.set_break(True)
    else:
        uart.write_data(LO)
        
    start = time.monotonic()
    while (time.monotonic() <= start + 0.0245):
       time.sleep(0.00025)

    if USE_BREAK:
        uart.set_break(False)
    else:
        uart.write_data(HI)

    start = time.monotonic()
    while (time.monotonic() <= start + 0.0245):
         time.sleep(0.00025)

    # Disable Bitbang Mode
    uart.set_bitmode(0x00, 0x00)
    uart.purge_buffers()

    INIT_FRAME = bytearray([0x81, 0x13, 0xF7, 0x81, 0x0C])
    START_DIAGNOSTICS = bytearray([0x02, 0x10, 0xA0, 0xB2])
    REQUEST_SEED = bytearray([0x02, 0x27, 0x01, 0x2A])
    KEY_RETURN = bytearray([0x04, 0x27, 0x02, 0x00, 0x00, 0x00])

    # >> 81 13 F7 81 0C
    # << 81 13 F7 81 0C 03 C1 57 8F AA
    # >> 02 10 A0 B2
    # << 02 10 A0 B2 01 50 51
    # >> 02 27 01 2A
    # << 02 27 01 2A 04 67 01 52 25 E3
    # >> 04 27 02 14 89 CA
    # << 04 27 02 14 89 CA 02 67 02 6B
        
    if USE_SERIAL:
        port = pyftdi.serialext.serial_for_url('ftdi://0x403:0x6001/1', baudrate=10400, bytesize=8, stopbits=1, parity='N')
        port.timeout = 0.5

    logit(INIT_FRAME, True);
    if USE_SERIAL:
        port.write(INIT_FRAME)
        response = port.read(READ_BUFFER_SIZE)
    else:
        uart.write_data(INIT_FRAME)
        response = uart.read_data(READ_BUFFER_SIZE)
    time.sleep(0.030)
    logit(response, False)

    logit(START_DIAGNOSTICS, True);
    if USE_SERIAL:
        port.write(START_DIAGNOSTICS)
        response = port.read(READ_BUFFER_SIZE)
    else:
        uart.write_data(START_DIAGNOSTICS)
        response = uart.read_data(READ_BUFFER_SIZE)
    time.sleep(0.030)
    logit(response, False)


    logit(REQUEST_SEED, True);
    if USE_SERIAL:
        port.write(REQUEST_SEED)
        response = port.read(READ_BUFFER_SIZE)
    else:
        uart.write_data(REQUEST_SEED)
        response = uart.read_data(READ_BUFFER_SIZE)

    # seed = response[7] << 8 | response[8]
    seed = 0x52 << 8 | 0x25
    key_hi, key_lo = calculate_key(seed)
    KEY_RETURN[3] = key_hi
    KEY_RETURN[4] = key_lo
    set_checksum(KEY_RETURN)

    logit(KEY_RETURN, True);
    if USE_SERIAL:
        port.write(KEY_RETURN)
        response = port.read(READ_BUFFER_SIZE)
    else:
        uart.write_data(KEY_RETURN)
        response = uart.read_data(READ_BUFFER_SIZE)
    
    time.sleep(0.030)
    logit(response, False)

    # if the ECU liked the key we should get
    # << 02 67 02 6B



    if USE_SERIAL:
        port.close()

    uart.close()

################################################################################
if __name__ == "__main__":
################################################################################
    fast_init()
      

