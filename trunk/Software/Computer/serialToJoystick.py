# -*- coding: utf-8 -*-
import serial
import os
import time, math

#debugData = "" # TODO: remove (debug)
packetCounter = 0


# Default command-line argument values
controllerName = None
numChannels = 0
arduinoDevice = None
doubleSweep = False
useCompositePPM = False
useDummyJoystick = False

maxLatency = 100e-3 # 100ms
doFiltering = True
filteringTolerance = 8

PPM_Period = 22e-3 # 22ms



def connectToArduino():
	# TODO: make this OS independant
	global arduinoDevice
	def fail():
		print ("Could not automatically find your Arduino device.")
		return None
	if not arduinoDevice:
		try:
			from serial.tools import list_ports
			dmesgOutput = os.popen("dmesg").read()
			lastOccurence = dmesgOutput.rfind("Arduino")
			if lastOccurence == -1:
				return fail()
			ttyBegin = dmesgOutput.find("tty", lastOccurence)
			if ttyBegin == -1:
				return fail()
			ttyEnd = min([dmesgOutput.find(c, ttyBegin) for c in (':', ' ', '\t', '\r', '\n') if c in dmesgOutput[ttyBegin:]])
			arduinoDevice = list(list_ports.grep(dmesgOutput[ttyBegin:ttyEnd]))[0][0]
		except:
			return fail()
	
	ser = serial.Serial(
			port = arduinoDevice,
			baudrate = 9600,
			bytesize = serial.EIGHTBITS,
			parity = serial.PARITY_NONE,
			stopbits = serial.STOPBITS_ONE,
			timeout = 1,
			xonxoff = 0,
			rtscts = 0,
			interCharTimeout = None
		)
	
	print ("Connected to Arduino at '%s'." % arduinoDevice)
	
	return ser


def connectToRCreceiver(ser):
	global numChannels
	command = (numChannels << 2) + (doubleSweep << 1) + useCompositePPM
	
	numRetries = 4
	for i in range(numRetries):
		ser.write(chr(command) + "\n")
		response = ser.readline()
		valid = (len(response) == 1 + 2 and response[1:3] == "\r\n")
		if not valid:
			continue
		response = ord(response[0])
		if numChannels:
			# Manual channel amount
			valid = (response == command)
			if not valid:
				continue
		else:
			# Automatic channel amount
			numChannels_received = response >> 2
			doubleSweep_received = response & 2
			useCompositePPM_received = response & 1
			valid = ((doubleSweep_received, useCompositePPM_received) == (doubleSweep, useCompositePPM))
			if not valid:
				continue
			valid = (1 <= numChannels_received <= 8)
			if valid:
				numChannels = numChannels_received
				print ("Autodetected %s channel(s)." % numChannels)
			else:
				if not numChannels_received:
					print ("Autodetection of amount of channels failed, \nmake sure you're RC transmitter is powered on.")
				else:
					print ("Autodetection of amount of channels is assumed to be faulty, since it's greater that 8.")
		break
	
	if not valid:
		print ("Failed to connect to RC receiver via Arduino on '%s'." % arduinoDevice)
		return False
	
	print ("Connected to RC receiver. (%s channels via %s)" % \
			(numChannels, ("channel connectors", "battery connector")[useCompositePPM]))
	
	return True


class Joystick:
	def __init__(self, numAxis, name):
		self.numAxis = numAxis
		self.name = name
	def emit(self, axisId, value, syn = True):
		raise NotImplementedError("This is not yet implemented.")

class UInputJoystick(Joystick): # Works on Linux
	def __init__(self, numAxis, name):
		Joystick.__init__(self, numAxis, name)
		import uinput
		events = [(3, axisId) + (0, 255, 0, 0) for axisId in range(self.numAxis)]
		self.joy = uinput.Device(events, name = self.name, bustype = 0x03) # bustype = BUS_USB
		#writeJoystick(self, chr(128) * numAxis) # reset joystick controls
	def emit(self, axisId, value, syn = True):
		self.joy.emit((3, axisId), value, syn) # uinput.ABS_Ch(axisId = (3, axisId)

class DummyJoystick(Joystick): # Platform independent, but no externally usable virtual joystick
	def __init__(self, numAxis, name):
		Joystick.__init__(self, numAxis, name)
		self.buffer = {}
	def emit(self, axisId, value, syn = True):
		self.buffer[axisId] = value
		if syn:
			out = "#### Incoming Joystick '%s' Event: ####\n" % self.name
			for axisId in sorted(list(self.buffer)):
				v = self.buffer[axisId]
				p = int(round(v / 2.55))
				meter = list("--------+--------")
				meter[int(round(v / 16.))] = 'o'
				out += "# Axis %s [%s] %s (%s%%)\n" % (axisId, ''.join(meter), v, p)
				del self.buffer[axisId]
			print (out)

def createJoystick():
	global controllerName
	if not controllerName:
		controllerName = "RC Receiver (%schs)" % numChannels
	
	def createUInputJoystick():
		joy = UInputJoystick(numChannels, name = controllerName)
		print ("Created a joystick device named '%s'." % controllerName)
		return joy
	def createDummyJoystick(fallback = False):
		joy = DummyJoystick(numChannels, name = controllerName)
		print ("I'll create a dummy one named '%s', it will print all inputs." % controllerName)
		if fallback:
			raw_input ("\nPress ENTER to continue. ")
		return joy
	
	if useDummyJoystick:
		return createDummyJoystick()
	
	try:
		return createUInputJoystick()
	except OSError:
		print ("Failed to create a joystick device,")
		return createDummyJoystick(True)


def readRCreceiver(ser, joy):
	#global debugData # TODO: remove (debug)
	global packetCounter
	buffer = ""

	dataSize = numChannels + 1 # 1: sep = '\xff'
	maxBuffer = math.ceil(dataSize * maxLatency/PPM_Period)

	while True:
		if ser.inWaiting() > maxBuffer:
			ser.flushInput()
			buffer = ""
			print ("Buffer overrun occured. Resetting buffer.")
		
		#time.sleep(20e-3 / 2) # 20ms / 2
		if ser.inWaiting():
			newData = ser.read(ser.inWaiting())
		else:
			newData = ser.read() # block cpu
		buffer += newData
		#debugData += newData # TODO: remove (debug)
		if '\xff' in buffer:
			data, buffer = buffer.split('\xff')[-2:]
			if len(data) == numChannels:
				writeJoystick(joy, processData(data))
				packetCounter += 1
			#print (data) # TODO: remove (debug)


def initFilter():
	global lastlastData, lastData, holdLastData
	lastlastData = numChannels * [127]
	lastData     = numChannels * [127]
	holdLastData = numChannels * [False]


def processData(data):
	global lastlastData, lastData, holdLastData
	data = list(data)
	#unf = list(data)
	for i, d in enumerate(data):
		data[i] = ord(d) - 1
		if data[i] >= 253:
			data[i] = 255
		#unf[i] = data[i]
		# Filtering of arduinos timing granularity at a frequency of about 45Hz
		if doFiltering:
			if holdLastData[i]:
				holdLastData[i] = (data[i] == lastData[i])
				if abs(data[i] - lastData[i]) < filteringTolerance:
					data[i] = lastData[i]
			elif data[i] == lastlastData[i]:
				holdLastData[i] = True
	lastlastData = lastData
	lastData = data
	#print (data, unf)
	return data


def writeJoystick(joy, data):
	for i in range(numChannels):
		joy.emit(i, data[i], syn = (i == numChannels-1))


def main():
	ser = joy = None
	
	def finalize():
		endTime = time.time()
		
		if ser:
			ser.close()
		
		#print (debugData.split("\xff")) # TODO: remove this
		#packetCounter = len(debugData.split("\xff")) # TODO: remove this
		if joy:
			#print ("Time of CPU:", endTime - startTime) # TODO: remove this
			expectedNbPackets = (endTime-startTime) / PPM_Period
			print ("%s packets captured (%s%%)" % (packetCounter, int(100 * packetCounter/expectedNbPackets)))
		
		print ("Connections closed.")

	try:
		ser = connectToArduino()
	except serial.serialutil.SerialException:
		print ("Failed to connect to Arduino on '%s'." % arduinoDevice)
		return finalize()

	if ser and connectToRCreceiver(ser):
		joy = createJoystick()
		if joy:
			initFilter()
			startTime = time.time()
			try:
				readRCreceiver(ser, joy)
			except serial.serialutil.SerialException:
				print ("Arduino device has been disconnected.")
			except KeyboardInterrupt:
				pass
			finally:
				return finalize()

if __name__ == "__main__":
	import argparse

	parser = argparse.ArgumentParser(description = 'Creates a virtual joystick controller using data from a RC receiver via an Arduino. '+ 
		'Go to http://code.google.com/p/rx-joystick-arduino/ for more info.')
		
	parser.add_argument('-c, --num-channels', dest = 'numChannels', action = 'store',
		default = numChannels, type = int, choices = xrange(0, 1+8),
		help = "the exact amount of channels used, if not provided or '0' is used, autodetection will be performed")

	parser.add_argument('-n, --name', dest = 'controllerName', action = 'store',
			default = controllerName,
			help = 'the name of the virtual joystick, how it will appear to other applications, if not provided, autogeneration will occur')
	parser.add_argument('-d, --device', dest = 'arduinoDevice', action = 'store',
			default = arduinoDevice,
			help = "the device path of the Arduino, on Linux usually something like '/dev/ttyACM0', if not provided, autodetection will be performed")

	parser.add_argument('-p, --use-ppm', dest = 'useCompositePPM', action='store_true',
			help = 'use one PPM stream as input to capture all channels, instead of using multiple channel streams')
	parser.add_argument('-D, --double-sweep', dest = 'doubleSweep', action = 'store_true',
			help = 'double the range of all channels, resolution will be halved')
	parser.add_argument('-t, --test', dest = 'useDummyJoystick', action = 'store_true',
			help = "print and visualise every controller event update, don't create an externally accessible virtual joystick")

	args = parser.parse_args()

	numChannels = args.numChannels
	controllerName = args.controllerName
	arduinoDevice = args.arduinoDevice
	useCompositePPM = args.useCompositePPM
	doubleSweep = args.doubleSweep
	useDummyJoystick = args.useDummyJoystick
	
	

	if useCompositePPM:
		print ("PPM input is not yet implemented. Using channels instead.")
		useCompositePPM = False

	main()
