DTM Python Readme 0.7.1

0. Changelog
1. Requrements
2. Contents
3. Installation
4. Usage
5. Reference

0.Changelog
---------------

v 0.7.1 - August 2013
* Fixed a bug where the RX tests practically ran the TX tests
* Added possibility to set a delay before tests to ensure that the firmware keeps up with the optional TestPause setup parameter

v.0.7.0 - March 2013
Initial release

1. Requirements
---------------
The DTM library is written in Python. It is tested with Python 2.7.2 with pySerial 2.6. It should be working with any Python 2 version from 2.7.

Python may be downloaded from http://www.python.org/getit/
pySerial may be downloaded from https://pypi.python.org/pypi/pyserial

2. Contents
-----------
This package contains:
* README.txt (this file)
* dtm.py (the library)
* example.py (some example usage)

3. Installation
---------------
The installation of the library is done by copying the dtm.py to the folder which contain the script(s) using the library.

4. Usage
--------
For examples on using the DTM library, see the file example.py.

Start by importing the library:
import dtm

4.1 Setup
---------
The project uses a setup dictionary to set up all the parameters.

4.1.1 TestSerialPortName - Required
-----------------------------------
This is the name of the serial port connected to the DUT (Device Under Test).

4.1.2 GoldenSerialPortName - Required
-------------------------------------
This is the name of the serial port connected to the Golden device.

4.1.3 Frequency - Required
--------------------------
The channel the test is to run on. From 0x00 to 0x27. See [Core] part F (Direct Test Mode), section 3.3.2 Commands for details

4.1.4 Bitpattern - Required
---------------------------
The packet type which should be used. See [Core] part F (Direct Test Mode), section 3.3.2 Commands for details

Available bitpatterns:
dtm.PRBS9				
dtm.FOUR_ONE_FOUR_ZERO	
dtm.ONE_ZERO			
dtm.CONSTANT_CARRIER	

4.1.4 Length - Required
-----------------------
The length of the data packages. From 0x00 to 0x25. See [Core] part F (Direct Test Mode), section 3.3.2 Commands for details

4.1.5 Runtime - Required
------------------------
The wanted test time in milliseconds.

4.1.6 PERLimit - Optional
-------------------------
The limit for the Packet Error Rate before an exception is thrown. Default 30.

4.1.7 TestPause - Optional
--------------------------
The wanted delay between tests in milliseconds. Default 0

4.1.8 Example usage
-------------------
setup = dict ()
setup['TestSerialPortName'] = "COM1"	#DUT (Device Under Test)
setup['GoldenSerialPortName'] = "COM3"	#Golden sample
setup['Frequency'] = 5					#Frequency the test should be run on
setup['Bitpattern'] = dtm.PRBS9			#The bit pattern used for the test. PRBS9, 11110000 (FOUR_ONE_FOUR_ZERO), 10101010 (ONE_ZERO), CONSTANT_CARRIER
setup['Length'] = 10					#Length of the test package
setup['Runtime'] = 100					#How long the Receiver should be receiving. In ms
setup['PERLimit'] = 30					#The packet error limit in percent. Default 30

4.2 Creating an object
----------------------
An object is created by sending the setup dictionary to the DTM constructor:
dtmObject = dtm.DTM(setup)

4.3 Running tests
-----------------

4.3.1 Running Transmitter (TX) tests
------------------------------------
try:
	txper = dtmObject.runTransmitterTest()
except dtm.DTMError as error:
	print error.errormessage()

4.3.1 Running Receiver (RX) tests
------------------------------------
try:
	txper = dtmObject.runReceiverTest()
except dtm.DTMError as error:
	print error.errormessage()
	
4.3.2 Running both Transmitter and Receiver tests
-------------------------------------------------
try:
	dtmObject.runBothRXandTXTests()
except dtm.DTMError as error:
	print error.errormessage()
	
print "TX PER: %s%%" % dtmObject.txper
print "RX PER: %s%%" % dtmObject.rxper

4.4 Exceptions
--------------
The library may throw some exceptions.

4.4.1 DTMError
--------------
Base error. Extends Exception. Call errormessage on this or any of the other exceptions for details.

4.4.2 ConnectionError
---------------------
Thrown if there is an error with the serial connection

4.4.3 MessageError
------------------
Thrown if there is an error with a sent or received package

4.4.4 PERError
--------------
Thrown if the PER for any test exceeds the set PERLimit.

5. Reference
------------
Core: Bluetooth specification 4.0 - Volume 6 - Core System Package [Low Energy Controller volume]