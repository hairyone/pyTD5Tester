# pyTD5Tester

A simple python script that can be run Python 3.6 under Windows to communicate with the Land Rover TD5 engine ECU.

To use this you will need a VAG COM KKL usb -> OBD2 cable that uses an FTDI chip.

Other software you will need:

* Python 3.6 for Windows, https://www.python.org/downloads/windows
* PyFtdi python library, https://github.com/eblot/pyftdi
* PyFtdi has dependencies on  PySerial and PyUsb which should automatically be installed when you install PyFtdi.
* PyFTdi requires the native library libusb (1.0.21 or above), https://github.com/libusb/wiki
* On Windows it is neceessary to install a device driver that sits on top of the libusb DLL, this will add the FTDI cable as a virtual COM port. The simplest way to do this is to use the Zadig utility from here: http://zadig.akeo.ie.

## Installation
On windows:
- `pip install -r requirements.txt` will install python dependencies
- Copy `libusb-1.0.dll` to `C/Windows/System32`
- Download https://zadig.akeo.ie/ and click install driver for the `FT232R USB UART`

On Linux:
- `pip install -r requirements.txt` will install python dependencies
