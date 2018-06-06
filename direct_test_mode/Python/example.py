# Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
#
# The information contained herein is property of Nordic Semiconductor ASA.
# Terms and conditions of usage are described in detail in NORDIC
# SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
#
# Licensees are granted free, non-transferable use of the information. NO
# WARRANTY of ANY KIND is provided. This heading must NOT be removed from
# the file.

import vdtm

def main():
	#General setup
	setup = dict ()
	setup['TestSerialPortName'] = "COM4"	#DUT (Device Under Test)
	setup['GoldenSerialPortName'] = "COM65"	#Golden sample
	setup['Frequency'] = 1					#Frequency the test should be run on
	setup['Bitpattern'] = vdtm.PRBS9			#The bit pattern used for the test. PRBS9, 11110000 (FOUR_ONE_FOUR_ZERO), 10101010 (ONE_ZERO), CONSTANT_CARRIER
	setup['Length'] = 10					#Length of the test package
	setup['Runtime'] = 100					#How long the Receiver should be receiving. In ms
	setup['PERLimit'] = 30					#The packet error limit in percent. Default 30
	vdtmObject = vdtm.DTM(setup)
	
	#Run tests on a series of channels	
	channels = [39]#, 5, 7, 9]
	txResult = 0
	
	print ""
	print "Running over a series of channels"
	
	for channel in channels:
		vdtmObject.changeChannel(channel)
		
		try:
			vdtmObject.runTransmitterTest()
			txResult = txResult + vdtmObject.txper
		except vdtm.DTMError as error:
			print error.errormessage()
			txResult = txResult + 100

		print "Results for channel: %s" % channel
		print "TX PER: %s%%" % vdtmObject.txper
		
	print "Total result:"
	print "TX PER: %s%%" % (txResult / len(channels))

	#vdtmObject.stopTest()
		
	del vdtmObject

if __name__ == '__main__':
    main()
