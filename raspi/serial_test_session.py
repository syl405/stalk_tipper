import serial
import time
import json

#======================
#ESTABLISH SERIAL COMMS
#======================
start = time.time()																#save start time
while True:																	
	try:
		arduinoSer = serial.Serial('/dev/ttyUSB1',115200)						#no timeout specified, need to handle silent line in rest of code
	except:
		if time.time() - start > 30:
			raise IOError("Serial initialization timeout, check Arduino")		#raise error if fail to initialize serial comms after 30s
	finally:
		print "Serial comms established"
		break																	#break out of loop once serial comms established


def listenAndWait(serObj, keyword, timeOut, n_compare=-1):
#*******************************************************************************
#Takes a serial object and listens for a keyword for a duration (specified in s)
#immediately returns True once keyword received; returns False if keyword
#not received at the end of specified duration. User can specify to compare for
#exact match or to compare the n first characters in a line.
#*******************************************************************************
	serObj.flushInput()
	if n_compare < 0:															#if n_compare not specified, default to comparing for exact match
		n_compare = len(keyword)
	keyword = keyword[0:n_compare]											#automatically truncate unnecessarily long keyword to save memory
	startTime = time.time()
	while time.time() - startTime < timeOut:									#check if specified duration has elapsed
		lineIn = serObj.readline()[0:n_compare]									#read from serial port and truncate to number of characters to compare
		if lineIn == keyword:
			return True															#immediately return True if keyword detected
	return False



#=======================================================
#MAIN LOOP, ONE ITERATION PER READY>TEST>CONFIRM>WRITE CYCLE
#=======================================================
while True: 																	#master loop, one iteration per initialize>test>write cycle
	#------------------------------------
	#CHECK THAT ARDUINO IS WAITING FOR TEST
	#------------------------------------
	if listenAndWait(arduinoSer, "WAITING", 10, 7) != True:
		raise IOError("Timeout: Arduino not waiting for test")					#raise exception if "WAITING" not received after 10s
	
	#----------------------------
	#SEND READY SIGNAL TO ARDUINO
	#----------------------------
	arduinoSer.write("READY")

	
	#----------------------------------------------------------------------
	#CHECK THAT ARDUINO IS READY FOR TEST AND INITIALISE TEST DATA VARIABLE
	#----------------------------------------------------------------------
	if listenAndWait(arduinoSer, "READY", 10, 5) != True:
		raise IOError("Timeout: Arduino not ready for test")					#raise exception if "READY" not received after 10s
	angleList = []
	loadList = []
	
	#--------------------------------------------------
	#WAIT FOR TEST BEGIN SIGNAL AND GET TEST IDENTIFIER
	#--------------------------------------------------
	if listenAndWait(arduinoSer, "BEGIN", 300, 5) !=  True:						#listen for test begin signal and require new handshake if test not started within 5 mins
		 continue
	idLine = arduinoSer.readline()												#read in line containing test ID number
	if idLine[0:7] != "TESTID=":
		raise IOError("Invalid test ID line received")
	else:
		testId = int(idLine[7:end])

	
	#---------------------------
	#SAVE TEST DATA TO VARIABLES
	#---------------------------
	lineReceived = arduinoSer.readline()										#read in next line from serial buffer
	while lineReceived[0:3] != "END":
		[loadReading, angleReading] = lineReceived.split(",")					#split load cell and potentiometer values using comma
		angleList.append(angleReading)											#append load and angle readings to lists
		loadList.append(loadReading)
	if idLine[0:7] != "TESTID=":
		raise IOError("Invalid test ID line received")
	elif int(idLine[7:end]) != testId:											#check if same test ID received at beginning and end of test
		raise IOError("Different test ID supplied at beginning and end of test")

	#-----------------------------------------------------------------------------
	#WAIT FOR USER CONFIRMATION AND WRITE TEST DATA TO FILE OR RETURN TO MAIN LOOP
	#-----------------------------------------------------------------------------
	lineReceived = arduinoSer.readline()										#read in next line from serial buffer (waiting until some signal is received)
	if lineReceived == "ACCEPT":
		testBivariateData = (loadList, angleList)								#place lists of load and angle into tuple of lists
		filename = "test" + char(testId).zfill(4) + ".test"						#formulate constant length filename based on test ID
		fileObj = open(filename,"w")											#open file to write
		json.dump(testBivariateData, fileObj)									#serialize and write bivariate test data to file
		fileObj.close()															#close file
	elif lineReceived == "REJECT":
		continue
	else:
		raise IOError("Invalid accept/reject instruction received")

