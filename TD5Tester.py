from pyftdi.ftdi import Ftdi
import time
from collections import namedtuple

CAUTIOUS_READ           = False
READ_BUFFER_SIZE        = 127

MAX_ATTEMPTS            = 5
ATTEMPT_DELAY           = 3     # 2s
SEND_REQUEST_DELAY      = 0.2   # 0.055 # 55ms
READ_RESPONSE_TIMEOUT   = 0.1   # 100ms

Pid = namedtuple('Pid', ['request', 'response_len'])

# http://www.rangerovers.net/forum/12-diagnostic-equipment/39941-working-homemade-error-reader-scanner-need-help-ecu-output.html
ABS_INIT_FRAME      = Pid(bytearray([0x81, 0x29, 0xF7, 0x81, 0x0C]),        7)

INIT_FRAME          = Pid(bytearray([0x81, 0x13, 0xF7, 0x81, 0x0C]),        7)
START_DIAGNOSTICS   = Pid(bytearray([0x02, 0x10, 0xA0, 0xB2]),              3)
REQUEST_SEED        = Pid(bytearray([0x02, 0x27, 0x01, 0x2A]),              6)
KEY_RETURN          = Pid(bytearray([0x04, 0x27, 0x02, 0x00, 0x00, 0x00]),  4)
BATTERY_VOLTAGE     = Pid(bytearray([0x02, 0x21, 0x10, 0x00]),              8)
ENGINE_RPM          = Pid(bytearray([0x02, 0x21, 0x09, 0x00]),              6)
VEHICLE_SPEED       = Pid(bytearray([0x02, 0x21, 0x0D, 0x00]),              5)

HI = bytearray([0x01])
LO = bytearray([0x00])

response    = None 
connected   = False

bit_mode_bitbang = Ftdi.BitMode(0x01)
bit_mode_reset = Ftdi.BitMode(0x00)

# set up the device
uart = Ftdi()


def pause(delay, step_size):
    end_time = time.monotonic() + delay
    while time.monotonic() <= end_time:
        time.sleep(step_size)


def calculate_checksum(request):
    request_len = len(request)
    crc = 0
    for i in range(0, request_len - 1):
        crc = crc + request[i]

    return crc % 256  # crc & 0xF


def log_data(data, is_tx):
    print("{} {}".format(
        ">>" if is_tx else "<<",
        ''.join('{:02X} '.format(x) for x in data).rstrip()
    ))


def read_data(size, timeout):
    data = bytearray()
    start = time.monotonic()
    while True:
        buf = uart.read_data(size)
        data += buf
        size -= len(buf)
        if size <= 0:
            break
        if timeout is not None:
            ms = time.monotonic() - start
            if ms > timeout:
                break
            time.sleep(0.01)

    return data


def get_pid(pid):
    global response
    
    result = False

    # Punch the calculated checksum into the last byte and then send the request
    request_len = len(pid.request)
    pid.request[request_len - 1] = calculate_checksum(pid.request)
    if not connected:
        log_data(pid.request, True)

    if pid != INIT_FRAME:
        pause(SEND_REQUEST_DELAY, 0.001)

    uart.write_data(pid.request)

    # read the response
    response = None        
    if CAUTIOUS_READ:
        response = read_data(READ_BUFFER_SIZE, 0.1)
    else:
        # The request is echoed in the response
        response = read_data(pid.response_len + request_len, 0.1)

    if not connected:
        log_data(response, False)

    # Remove the request from the response
    response = response[request_len:]

    # Check the response checksum
    response_len = len(response)
    if response_len > 1:
        cs1 = response[response_len - 1]
        cs2 = calculate_checksum(response)
        if cs1 == cs2:
            # Negative response ?
            if response[1] != 0x7F:
                result = True

    return result


def calculate_key(seed):
    count = ((seed >> 0xC & 0x8) + (seed >> 0x5 & 0x4) + (seed >> 0x3 & 0x2) + (seed & 0x1)) + 1

    for idx in range(0, count):
        tap = ((seed >> 1) + (seed >> 2) + (seed >> 8) + (seed >> 9)) & 1
        tmp = (seed >> 1) | (tap << 0xF)
        
        if (seed >> 0x3 & 1) and (seed >> 0xD & 1):
            seed = tmp & ~1
        else:
            seed = tmp | 1

    return seed >> 8, seed & 255


def open_uart():
    try:
        uart.open(0x403, 0x6001)
    except Exception as e:
        print("error={}".format(e))
        return
    
    uart.set_baudrate(10400)
    uart.set_line_property(8, 1, 'N')
    # print(uart.modem_status())


def slow_init(address):

    # Set K-line HI for 300ms
    # Transmit address byte at 5 baud (0x33)
    # Switch to 10400 baud
    # Wait 60-300ms for synchronisation pattern byte 0x55
    # Wait 5-20ms for KB1 (one of 0xE9 0x6B 0x6D 0xEF)
    # Wait 0-20ms for KB2 (always 0x8F)
    # Wait 25-50ms and send inverted KB2
    # Wait 25-50ms and send inverted address byte

    global response
    global connected

    if uart is None:
        return

    uart.set_bitmode(0x01, bit_mode_bitbang)
        
    # K line HI for 300ms
    uart.write_data(HI)
    pause(0.300, 0.001)

    # Start bit LO
    uart.write_data(LO)
    pause(0.200, 0.001)

    # Send the target address LSB first at 5 Baud
    for i in range(0, 8):
        hilo = address >> i & 0x01
        uart.write_data(hilo)
        pause(0.200, 0.001)

    # Stop bit HI
    uart.write_data(HI)
    pause(0.200, 0.01)

    # Switch off bit bang
    uart.set_bitmode(0x00, bit_mode_reset)
    uart.purge_buffers()

    # Wait up 300ms + 20ms + 20ms to read Sync + KB1 + KB2 bytes
    response = uart.read_data(3, 0.340)

    log_data(response, False)
    if response[0] == 0x55 and response[2] == 0x8F:
        inverted_address    = bytearray([~address])
        inverted_kb2        = bytearray([~response[2]])

        # Send inverted KB2
        pause(0.025, 0.001)
        uart.write_data(inverted_kb2)
        log_data(inverted_kb2)

        # Send inverted address
        pause(0.025, 0.001)
        uart.write_data(inverted_address)
        log_data(inverted_address)
        
        connected = True
    else:
        uart.close()


def fast_init():
    global KEY_RETURN
    global response
    global connected
    
    HI = bytearray([0x01])
    LO = bytearray([0x00])

    attempt = 0
    while attempt < MAX_ATTEMPTS:
        # Toggle the TX line for the fast_init using the ftdi chip bit bang mode
        uart.set_bitmode(0x01, bit_mode_bitbang)
        
        uart.write_data(HI)
        pause(0.500, 0.01)

        uart.write_data(LO)
        pause(0.0245, 0.00025)

        uart.write_data(HI)
        pause(0.0245, 0.00025)

        # Switch off bit bang
        uart.set_bitmode(0x00, bit_mode_reset)
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
            KEY_RETURN.request[3] = key_hi
            KEY_RETURN.request[4] = key_lo
            connected = get_pid(KEY_RETURN)

        if connected:
            break
        pause(ATTEMPT_DELAY, 0.01)
        attempt += 1

    # fast_init failed
    if not connected:
        uart.close()


def start_logger():
    if not connected:
        return

    start = time.monotonic()
    while True:
        buf = "{:010.3f}".format(time.monotonic() - start)
        
        if get_pid(BATTERY_VOLTAGE):
            buf += " " "Voltage: {:06.2f}".format((response[5] << 8 | response[6]) / 1000.0)

        if get_pid(ENGINE_RPM):
            buf += " " "Engine RPM: {:06d}".format(response[3] << 8 | response[4])

        if get_pid(VEHICLE_SPEED):
            buf += " " "Vehicule speed: {:03d}".format(response[3])
        
        print(buf)
        

if __name__ == "__main__":

    # TODO: Continuously wait to connect and handle ignition off and on
    # TODO: Start a new data file each time the ignition is switched on
    # TODO: See how fast we can reliably poll the ECU
    # TODO: Can you connect after the engine has started ?
    
    open_uart()
    fast_init()
    print('Fast init done !')
    start_logger()

