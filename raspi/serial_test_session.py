import serial

#======================
#ESTABLISH SERIAL COMMS
#======================
start = time.clock()															#save start time
while True																		
	try:
		arduinoSer = serial.Serial('/dev/ttyUSB0',115200)
	except:
		if time.clock() - start > 30:
			raise IOError("Serial initialization timeout, check Arduino")		#raise error if fail to initialize serial comms after 30s
	finally:
		break																	#break out of loop once serial comms established

#=======================================================
#MAIN LOOP, ONE ITER. PER READY>TEST>CONFIRM>WRITE CYCLE
#=======================================================
while True: 																	#master loop, one iteration per initialize>test>write cycle
	#------------------------------------
	#CHECK THAT ARDUINO IS WAITING FOR TEST
	#------------------------------------
	if !listenAndWait(arduinoSer, "WAITING", 10):
		raise IOError("Timeout: Arduino not waiting for test")					#raise exception if "WAITING" not received after 10s
	
	#----------------------------
	#SEND READY SIGNAL TO ARDUINO
	#----------------------------
	arduinoSer.write("READY")

	
	#------------------------------------
	#CHECK THAT ARDUINO IS READY FOR TEST
	#------------------------------------
	if !listenAndWait(arduinoSer, "READY", 10):
		raise IOError("Timeout: Arduino not ready for test")					#raise exception if "READY" not received after 10s

	

#TO-DO: WRITE FUNCTION TO LISTEN FOR SIGNAL FOR SPECIFIC NUMBER OF LOOPS ITERATIONS OR # OF SECONDS THEN REUSE CODE

def listenAndWait(serObj, keyword, timeOut):
#*******************************************************************************
#Takes a serial object and listens for a keyword for a duration (specified in s)
#immediately returns True once keyword received; returns False if keyword
#not received at the end of specified duration.
#*******************************************************************************
	startTime = time.clock()
	while time.clock() - startTime < timeOut:									#check if specified duration has elapsed
		lineIn = serObj.readline()												#read from serial port
		if lineIn == keyword:
			return True															#immediately return True if keyword detected
	return False


