import serial
import struct
import time
import os
import json
import numpy  # Import numpy
import matplotlib.pyplot as plt #import matplotlib library
from drawnow import *

def packIntegerAsULong(value):
    """Packs a python 4 byte unsigned integer to an arduino unsigned long"""
    return struct.pack('I', value)    #should check bounds

def makeFig(): #Create a function that makes our desired plot
#*******************************************************************************
#Live plotting function borrowed from example at http://www.toptechboy.com/tutorial/python-with-arduino-lesson-11-plotting-and-graphing-live-data-from-arduino-with-matplotlib/
#*******************************************************************************
    #plt.ylim(0,20000)                                 							#Set axes limits for load
    plt.title('My Live Streaming Sensor Data')      							#Plot the title
    plt.grid(True)                                  							#Turn the grid on
    plt.ylabel('Load')                            								#Set ylabels
    plt.plot(angleList, loadList, 'ro')       											#plot the load-deflection data
    plt.ticklabel_format(useOffset=True)          								#Force matplotlib to NOT autoscale y axis

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
lastTestId = 0																	#highest test ID amongst existing test files
if os.path.exists(dirPath + datePrefix) != True:								#check if directory for today already exists
	os.makedirs(dirPath + datePrefix)											#make directory
	print "Making new directory for today's test files"							#debug
elif os.path.isdir(dirPath + datePrefix):										#if directory already exists continue test numbering from last time
	testFileList = os.listdir(dirPath + datePrefix)								#list all elements in the existing directory
	for file in testFileList:
		if file[len(file)-4:len(file)] == "test":								#check if file is a test data file
			if int(file[4:8]) > lastTestId:										#check if current testID is greater than last greatest ID
				lastTestId = int(file[4:8])										#make current testID last greatest ID
	lastTestId += 1																#Arduino numbers tests from 0, so increment by 1 to prevent overwriting previous test
	print "Continuing from previous test files"									#debug


	

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
	print "test " + str(lastTestId + testId) + " started" #debug

	zeroLoadLine = arduinoSer.readline()
	if zeroLoadLine[0:7] != "NOLOAD=":
		raise IOError("Invalid zero load line received")
	else:
		zeroLoadCode = int(zeroLoadLine[7:len(zeroLoadLine)])

	zeroPotLine = arduinoSer.readline()
	if zeroPotLine[0:6] != "NOROT=":
		raise IOError("Invalid zero rotation line received")
	else:
		zeroPotCode = int(zeroPotLine[6:len(zeroPotLine)])

	heightLine = arduinoSer.readline()											#read in line containing force applicator height
	if heightLine[0:7] != "HEIGHT=":
		print heightLine #debug
		raise IOError("Invalid force applicator height received")
	else:
		height = int(heightLine[7:len(heightLine)])
	print "testing at " + str(height) + " mm above ground." #debug

	#------------------------------------
	#SAVE TEST DATA TO VARIABLES AND PLOT
	#------------------------------------
	lineReceived = arduinoSer.readline()										#read in first line of test data
	while lineReceived[0:3] != "END":
		lineReceived = lineReceived.split("/")[0]								#take first element after splitting by forward slash to strip special characters
		[loadReading, angleReading] = lineReceived.split(",")					#split load cell and potentiometer values using comma
		angleList.append(int(angleReading))										#append load and angle readings to lists
		loadList.append(int(loadReading))
	plt.ion()
	drawnow(makeFig)
	idLine = arduinoSer.readline()												#read in line following test end signal to get testID
	if idLine[0:7] != "TESTID=":
		raise IOError("Invalid test ID line received")
	elif int(idLine[7:len(idLine)]) != testId:									#check if same test ID received at beginning and end of test
		raise IOError("Different test ID supplied at beginning and end of test")
	print "test " + str(lastTestId + testId) + " ended" #debug

	arduinoSer.write("TESTIDONFILE=" + str(lastTestId + testId).zfill(4))		#send final test ID associated with file back to Arduino

	#-----------------------------------------------------------------------------
	#WAIT FOR USER CONFIRMATION AND WRITE TEST DATA TO FILE OR RETURN TO MAIN LOOP
	#-----------------------------------------------------------------------------
	lineReceived = arduinoSer.readline()										#read in next line from serial buffer (blocking until some signal is received)
	print lineReceived
	if lineReceived[0:6] == "ACCEPT":
		testData = (zeroLoadCode, zeroPotCode, height, loadList, angleList)		#place zeroLoadCode, zeroPotCode, height and lists of load and angle into tuple of lists
		filename = dirPath + datePrefix + "/test" + str(lastTestId + testId).zfill(4) + ".test"			#formulate constant length filename based on test ID (continue numbering from prev.)
		fileObj = open(filename,"w")											#open file to write
		json.dump(testData, fileObj)											#serialize and write test data to file
		fileObj.close()															#close file
		plt.close()		
		continue
	elif lineReceived[0:6] == "REJECT":
		plt.close()
		continue
	else:
		raise IOError("Invalid accept/reject instruction received")

