# -*- coding: utf-8 -*-
import serial
import os
import time, math, struct

packetCounter = 0


# Default command-line argument values
controllerName = None
numChannels = 0
arduinoDevice = None
useCompositePPM = -1
doubleSweep = False
useDummyJoystick = False
filteringFuzz = 6

maxLatency = 100e-3 # 100ms
periodFound = False # Measured period found
PPM_Period = 20e-3 # Default period: 20ms



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
	global numChannels, useCompositePPM, PPM_Period, periodFound
	command = (numChannels << 3) + (useCompositePPM << 2) + (modeAutodetect << 1) + doubleSweep
	
	numRetries = 5
	for i in range(numRetries):
		# Send command
		ser.write(chr(command) + "\xff")
		
		# Check response format
		response = ser.readline()
		valid = (len(response) == 1 + 4 + 2 and response[-2:] == "\r\n")
		if not valid:
			continue
		command_received = ord(response[0])
		numChannels_received = command_received >> 3
		useCompositePPM_received = (command_received & (1 << 2)) >> 2
		modeAutodetect_received = (command_received & (1 << 1)) >> 1
		doubleSweep_received = command_received & 1
		valid = (doubleSweep_received == doubleSweep)
		if not modeAutodetect:
			valid = (valid and useCompositePPM_received == useCompositePPM and modeAutodetect_received == modeAutodetect)
		if numChannels:
			valid = (valid and numChannels_received == numChannels)
		if not valid:
			continue
		
		# Check respons results
		if modeAutodetect: # Automatic mode
			valid = not modeAutodetect_received
			if valid:
				useCompositePPM = useCompositePPM_received
				print ("Autodetected %s mode." % ("separate PWM", "composite PPM")[useCompositePPM])
			else:
				print ("Autodetection of mode failed, \nmake sure you're RC transmitter (and receiver if using channel outputs,) is powered on.")
		if not numChannels: # Automatic channel amount
			valid = (1 <= numChannels_received <= 8)
			if valid:
				numChannels = numChannels_received
				print ("Autodetected %s channel(s)." % numChannels)
			else:
				if not numChannels_received:
					print ("Autodetection of amount of channels failed, \nmake sure you're RC transmitter%s is powered on." % (" (and receiver)", "")[useCompositePPM])
				else:
					print ("Autodetection of amount of channels is assumed to be faulty, since it's greater that 8.")
		else: # Manual channel amount
			pass
		
		# Store period
		period_received = struct.unpack('<L', response[1:1+4])[0]
		if period_received:
			periodFound = True
			PPM_Period = period_received * 1e-6
			print ("Detected refresh rate of %.0f Hz." % round(1./PPM_Period))
		
		break
	
	if not valid:
		print ("Failed to connect to RC transmitter/receiver via Arduino on '%s'." % arduinoDevice)
		return False
	
	print ("Connected to RC %s. (%s channel(s) via %s)" % \
			(("receiver", "transmitter")[useCompositePPM], numChannels, ("channel connectors", "D.S.C. connector")[useCompositePPM]))
	
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
		controllerName = "RC Controller (%s mode) (%schs)" % (("PWM", "PPM")[useCompositePPM], numChannels)
	
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
	global packetCounter
	buffer = ""

	dataSize = numChannels + 1 # 1: sep = '\xff'
	maxBuffer = math.ceil(dataSize * maxLatency/PPM_Period)

	while True:
		if ser.inWaiting() > maxBuffer:
			ser.flushInput()
			buffer = ""
			print ("Buffer overrun occured. Resetting buffer.")
		
		if ser.inWaiting():
			newData = ser.read(ser.inWaiting())
		else:
			newData = ser.read() # block cpu
		buffer += newData
		if '\xff' in buffer:
			data, buffer = buffer.split('\xff')[-2:]
			if len(data) == numChannels:
				writeJoystick(joy, processData(data))
				packetCounter += 1


class Filter:
	def __init__(self, fuzz):
		self.fuzz = fuzz
		self.left = self.right = None
	def process(self, data):
		if self.left == None: # and self.right == None:
			self.left = self.right = data
		elif max(data - self.left, self.right - data) >= self.fuzz:
			self.left = self.right = data
		elif data < self.left:
			self.left = data
			return data # Return unfiltered value to ensure quick response
		elif self.right < data:
			self.right = data
			return data # Return unfiltered value to ensure quick response
		return (self.left + self.right) / 2

def initFilter():
	global filters
	filters = [Filter(filteringFuzz) for i in range(numChannels)]


def processData(data):
	data = list(data)
	for i, d in enumerate(data):
		data[i] = ord(d) - 1
		if data[i] >= 253:
			data[i] = 255
		data[i] = filters[i].process(data[i])
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
		
		if joy:
			expectedNbPackets = (endTime-startTime) / PPM_Period
			if periodFound:
				print ("%s packets captured (%.0f%%)" % (packetCounter, round(100. * packetCounter/expectedNbPackets)))
		
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
			writeJoystick(joy, [127] * numChannels) # reset joystick controls
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
		
	parser.add_argument('-m, --mode', dest = 'useCompositePPM', action='store',
		default = useCompositePPM, type = int, choices = [-1, 0, 1],
		help = "the type of source we'll be capturing from: '0': use multiple separate PWM channel inputs (from receiver outputs); '1': use one combined PPM input (e.g. from transmitter's D.S.C. output) as input to capture all channels; if not provided or '-1' is used, autodetection will be performed, PWM mode has the highest priority")
	parser.add_argument('-c, --num-channels', dest = 'numChannels', action = 'store',
		default = numChannels, type = int, choices = range(0, 1+8),
		help = "the exact amount of channels used, if not provided or '0' is used, autodetection will be performed")
	parser.add_argument('-f, --filtering-fuzz', dest = 'filteringFuzz', action = 'store',
		default = filteringFuzz, type = int, choices = range(1, 1+16),
		help = "the number of adjacent values that are result of quantization noise, if '1' is used, no filtering will be performed; default value is '%s'" % filteringFuzz)

	parser.add_argument('-n, --name', dest = 'controllerName', action = 'store',
		default = controllerName,
		help = 'the name of the virtual joystick, how it will appear to other applications, if not provided, autogeneration will occur')
	parser.add_argument('-d, --device', dest = 'arduinoDevice', action = 'store',
		default = arduinoDevice,
		help = "the device path of the Arduino, on Linux usually something like '/dev/ttyACM0', if not provided, autodetection will be performed")

	parser.add_argument('-D, --double-sweep', dest = 'doubleSweep', action = 'store_true',
		help = "double the range of all channels, resolution will be halved")
	parser.add_argument('-t, --test', dest = 'useDummyJoystick', action = 'store_true',
		help = "print and visualise every controller event update, don't create an externally accessible virtual joystick")

	args = parser.parse_args()

	numChannels = args.numChannels
	controllerName = args.controllerName
	arduinoDevice = args.arduinoDevice
	useCompositePPM = args.useCompositePPM
	modeAutodetect = (useCompositePPM == -1)
	useCompositePPM = max(0, useCompositePPM)
	doubleSweep = args.doubleSweep
	useDummyJoystick = args.useDummyJoystick
	filteringFuzz = args.filteringFuzz
	
	
	main()
