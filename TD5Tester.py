from pyftdi.ftdi import Ftdi
import pyftdi.serialext
import time

READ_BUFFER_SIZE    = 127

INIT_FRAME          = bytearray([0x81, 0x13, 0xF7, 0x81, 0x0C])
START_DIAGNOSTICS   = bytearray([0x02, 0x10, 0xA0, 0xB2])
REQUEST_SEED        = bytearray([0x02, 0x27, 0x01, 0x2A])
KEY_RETURN          = bytearray([0x04, 0x27, 0x02, 0x00, 0x00, 0x00])
BATTERY_VOLTAGE     = bytearray([0x02, 0x21, 0x10, 0x00]);

response = None
connected = False
uart = None

################################################################################
def pause(ms):
################################################################################
    s = ms / 1000
    end_time = time.monotonic() + s
    while (time.monotonic() <= end_time):
        time.sleep(0.01)

################################################################################
def calculate_checksum(request):
################################################################################
    request_len = len(request)
    crc = 0
    for i in range(0, request_len - 1):
        crc = crc + request[i]

    # get leftmost byte, two ways
    # return crc & 0xF
    return crc % 256

################################################################################
def log_data(data, is_tx):
################################################################################
    print("{} {}".format(
        ">>" if is_tx else "<<",
        ''.join('{:02X} '.format(x) for x in data).rstrip()
    ))

################################################################################
def get_pid(request):
################################################################################
    global response
    
    result = False

    # Punch the calculated calculate_checksum into the last byte and then send the request
    request_len = len(request)
    request[request_len - 1] = calculate_checksum(request)
    log_data(request, True)

    # Send the request but wait a little to give the ECU a chance to respond to
    # the previous request
    pause(50)
    uart.write_data(request)

    # read the response
    response = uart.read_data(READ_BUFFER_SIZE)    
    log_data(response, False)

    # The request is echoed in the response so we slice it out just leaving the
    # actual response.
    response = response[request_len:]

    # Validate the calculate_checksum
    response_len = len(response)
    if response_len > 1:
        cs1 = response[response_len - 1]
        cs2 = calculate_checksum(response)
        if cs1 == cs2:
            # Negative response ?
            if response[1] != 0x7F:
                result = true

    return result

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
def fast_init():
################################################################################
    global uart
    
    HI = bytearray([0x01])
    LO = bytearray([0x00])

    # set up the device
    uart = Ftdi()
    uart.open(0x403, 0x6001)
    uart.set_baudrate(10400)
    uart.set_line_property(8, 1, 'N')

    # Toggle the TX line for the fast_init using the ftdi chip bit bang mode
    uart.set_bitmode(0x01, 0x01)
    
    uart.write_data(HI)
    pause(500)

    uart.write_data(LO)
    pause(25)

    uart.write_data(HI)
    pause(100)

    # Switch off bit bang
    uart.set_bitmode(0x00, 0x00)
    uart.purge_buffers()

    # Start communications

    # >> 81 13 F7 81 0C
    # << 81 13 F7 81 0C 03 C1 57 8F AA
    # >> 02 10 A0 B2
    # << 02 10 A0 B2 01 50 51
    # >> 02 27 01 2A
    # << 02 27 01 2A 04 67 01 52 25 E3
    # >> 04 27 02 14 89 CA
    # << 04 27 02 14 89 CA 02 67 02 6B

    if get_pid(INIT_FRAME) and get_pid(START_DIAGNOSTICS) and get_pid(REQUEST_SEED):
        seed = response[3] << 8 | response[4]
        key_hi, key_lo = calculate_key(seed)
        KEY_RETURN[3] = key_hi
        KEY_RETURN[4] = key_lo
        connected = get_pid(KEY_RETURN)

################################################################################
def start_logger():
################################################################################
    if not connected:
        return

    while True:
        
        if get_pid(BATTERY_VOLTAGE):
            print("Battery voltage: {:06.2f}".format(response[5] << 8 | response[6] / 1000.0))
        
################################################################################
if __name__ == "__main__":
################################################################################
    fast_init()
    start_logger()

