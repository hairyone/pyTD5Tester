from pyftdi.ftdi import Ftdi
import pyftdi.serialext
import time
from collections import namedtuple

CAUTIOUS_READ           = True
READ_BUFFER_SIZE        = 127
SEND_REQUEST_DELAY      = 55
READ_RESPONSE_TIMEOUT   = 100

Request = namedtuple('Request', ['request', 'expected_len'])

X_INIT_FRAME            = Request(bytearray([0x81, 0x13, 0xF7, 0x81, 0x0C]),        7)
X_START_DIAGNOSTICS     = Request(bytearray([0x02, 0x10, 0xA0, 0xB2]),              3)
X_REQUEST_SEED          = Request(bytearray([0x02, 0x27, 0x01, 0x2A]),              6)
X_KEY_RETURN            = Request(bytearray([0x04, 0x27, 0x02, 0x00, 0x00, 0x00]),  4)
X_BATTERY_VOLTAGE       = Request(bytearray([0x02, 0x21, 0x10, 0x00]),              8)
X_ENGINE_RPM            = Request(bytearray([0x02, 0x21, 0x09, 0x00]),              6)

response = None 
connected = False   
uart = None
port = None

################################################################################
def pause(delay, step_size):
################################################################################
    end_time = time.monotonic() + delay
    while (time.monotonic() <= end_time):
        time.sleep(step_size)

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
def get_pid(request, expected_len):
################################################################################
    global response
    
    result = False

    # Punch the calculated checksum into the last byte and then send the request
    request_len = len(request)
    request[request_len - 1] = calculate_checksum(request)
    if not connected:
        log_data(request, True)

    # Send the request but wait a little to give the ECU a chance to respond to
    # any previous request
    # pause(SEND_REQUEST_DELAY / 1000, 0.0025)
    port.write(request)

    pause(0.055, 0.01)
    # read the response
    response = None
    if CAUTIOUS_READ:
        response = port.read(READ_BUFFER_SIZE)
    else:
        response = port.read(expected_len)
        
    if not connected:
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
                result = True

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
    global port
    global X_KEY_RETURN
    global response
    global connected
    
    HI = bytearray([0x01])
    LO = bytearray([0x00])

    # set up the device
    uart = Ftdi()
    try:
        uart.open(0x403, 0x6001)
    except Exception as e:
        print("error={}".format(e))
        return
    
    uart.set_baudrate(10400)
    uart.set_line_property(8, 1, 'N')

    # Toggle the TX line for the fast_init using the ftdi chip bit bang mode
    uart.set_bitmode(0x01, 0x01)
    
    uart.write_data(HI)
    pause(0.500, 0.01)

    uart.write_data(LO)
    pause(0.0245, 0.00025)

    uart.write_data(HI)
    pause(0.100, 0.00025)

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

    port = pyftdi.serialext.serial_for_url('ftdi://0x403:0x6001/1', baudrate=10400, bytesize=8, stopbits=1, parity='N')
    port.timeout = READ_RESPONSE_TIMEOUT / 1000

    if (    get_pid(X_INIT_FRAME.request, X_INIT_FRAME.expected_len)
        and get_pid(X_START_DIAGNOSTICS.request, X_START_DIAGNOSTICS.expected_len)
        and get_pid(X_REQUEST_SEED.request, X_REQUEST_SEED.expected_len)
        ):
        seed = response[3] << 8 | response[4]
        key_hi, key_lo = calculate_key(seed)
        X_KEY_RETURN.request[3] = key_hi
        X_KEY_RETURN.request[4] = key_lo
        connected = get_pid(X_KEY_RETURN.request, X_KEY_RETURN.expected_len)

################################################################################
def start_logger():
################################################################################
    if not connected:
        return

    while True:
        
        if get_pid(X_BATTERY_VOLTAGE.request, X_BATTERY_VOLTAGE.expected_len):
            print("=={:>20}: {:06.2f}".format(
                "Battery voltage",
                (response[5] << 8 | response[6]) / 1000.0
            ))

        if get_pid(X_ENGINE_RPM.request, X_ENGINE_RPM.expected_len):
            print("=={:>20}: {:06d}".format(
                "Engine RPM",
                (response[3] << 8 | response[4])
            ))
        
################################################################################
if __name__ == "__main__":
################################################################################
    fast_init()
    start_logger()

