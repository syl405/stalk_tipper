import serial

#======================
#ESTABLISH SERIAL COMMS
#======================
start = time.clock()															#save start time
while True																		
	try:
		serobj = serial.Serial('/dev/ttyUSB0',115200)
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
	#CHECK THAT ARDUINO IS READY FOR TEST
	#------------------------------------
	line = serobj.readline() 													#read in and store line from arduino
	if line == "WAITING": 														#if waiting signal received
		serobj.write("READY") 													#send ready signal
		arduino_ready = False 													#boolean for arduino state
		
		while !arduino_ready:
			line = serobj.readline():
			if line == "READY": 												#arduino returns ready signal
				arduino_ready = True; 											#toggle boolean to true and exit loop
			#TO-DO: RAISE EXCEPTION IF TIMEOUT

	

#TO-DO: WRITE FUNCTION TO LISTEN FOR SIGNAL FOR SPECIFIC NUMBER OF LOOPS ITERATIONS OR # OF SECONDS THEN REUSE CODE

def listenAndWait(serobj, keyword, seconds)
#*******************************************************************************
#Takes a serial object and listens for a keyword for a duration (specified in s)
#immediately returns True once keyword received; returns False if keyword
#not received at the end of specified duration.
#*******************************************************************************
		
