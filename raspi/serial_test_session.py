import serial
import time
import os
import json

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
	keyword = keyword[0:n_compare]												#automatically truncate unnecessarily long keyword to save memory
	startTime = time.time()
	while time.time() - startTime < timeOut:									#check if specified duration has elapsed
		lineIn = serObj.readline()[0:n_compare]									#read from serial port and truncate to number of characters to compare
		if lineIn == keyword:
			return True															#immediately return True if keyword detected
	return False

#======================
#ESTABLISH SERIAL COMMS
#======================
start = time.time()																#save start time
while True:																	
	try:
		arduinoSer = serial.Serial('/dev/ttyUSB0',115200)						#no timeout specified, need to handle silent line in rest of code
		break
	except:
		if time.time() - start > 10:
			raise IOError("Serial initialization timeout, check Arduino")		#raise error if fail to initialize serial comms after 30s
print "Serial comms established"
#	finally:
#		print "Serial comms established"
#		break																	#break out of loop once serial comms established
#	return False																#return false if serial comms not established after pre-specified time

#=============================================
#SETUP DIRECTORY STRUCTURE TO STORE TEST FILES
#=============================================
dirPath = "test_data"															#literal specifying location for all test data
datePrefix = time.strftime("%d%m%y")											#get system date as a string
lastTestID = 0																	#highest test ID amongst existing test files
if os.path.exists(dirPath + datePrefix) != True:								#check if directory for today already exists
	os.makedirs(dirPath + datePrefix)											#make directory
elif os.path.isdir(dirPath + datePrefix):										#if directory already exists continue test numbering from last time
	testFileList = os.listdir(dirPath + datePrefix)								#list all elements in the existing directory
	for file in testFileList:
		if file[len(file)-4:len(file)] == test:									#check if file is a test data file
			if int(file[4:8]) > lastTestID:										#check if current testID is greater than last greatest ID
				lastTestID = int(file[4:8])										#make current testID last greatest ID
	lastTestID += 1																#Arduino numbers tests from 0, so increment by 1 to prevent overwriting previous test


	

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
	print "READY SIGNAL SENT" #debug

	
	#----------------------------------------------------------------------
	#CHECK THAT ARDUINO IS READY FOR TEST AND INITIALISE TEST DATA VARIABLE
	#----------------------------------------------------------------------
	if listenAndWait(arduinoSer, "READY", 10, 5) != True:
		raise IOError("Timeout: Arduino not ready for test")					#raise exception if "READY" not received after 10s

	print "Arduino ready for test" #debug
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
		testId = int(idLine[7:len(idLine)])
	print "test " + str(testId) + " started" #debug

	#---------------------------
	#SAVE TEST DATA TO VARIABLES
	#---------------------------
	lineReceived = arduinoSer.readline()										#read in first line of test data
	while lineReceived[0:3] != "END":
		lineReceived = lineReceived.split("/")[0]								#take first element after splitting by forward slash to strip special characters
		[loadReading, angleReading] = lineReceived.split(",")					#split load cell and potentiometer values using comma
		angleList.append(int(angleReading))										#append load and angle readings to lists
		loadList.append(int(loadReading))
		lineReceived = arduinoSer.readline()									#read in next line of test data
	idLine = arduinoSer.readline()												#read in line following test end signal to get testID
	if idLine[0:7] != "TESTID=":
		raise IOError("Invalid test ID line received")
	elif int(idLine[7:len(idLine)]) != testId:									#check if same test ID received at beginning and end of test
		raise IOError("Different test ID supplied at beginning and end of test")
	print "test " + str(testId) + " ended" #debug

	#-----------------------------------------------------------------------------
	#WAIT FOR USER CONFIRMATION AND WRITE TEST DATA TO FILE OR RETURN TO MAIN LOOP
	#-----------------------------------------------------------------------------
	lineReceived = arduinoSer.readline()										#read in next line from serial buffer (blocking until some signal is received)
	print lineReceived
	if lineReceived[0:6] == "ACCEPT":
		testBivariateData = (loadList, angleList)								#place lists of load and angle into tuple of lists
		filename = "test" + str(lastTestID + testId).zfill(4) + ".test"			#formulate constant length filename based on test ID (continue numbering from prev.)
		fileObj = open(filename,"w")											#open file to write
		json.dump(testBivariateData, fileObj)									#serialize and write bivariate test data to file
		fileObj.close()															#close file
	elif lineReceived[0:6] == "REJECT":
		continue
	else:
		raise IOError("Invalid accept/reject instruction received")

