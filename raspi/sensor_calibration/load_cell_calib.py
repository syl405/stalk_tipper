import serial
import time
import os
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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

def data_gen():
	i = 0;
	while i < 10000:
		#---------------------------
		#SAVE TEST DATA TO VARIABLES
		#---------------------------
		lineReceived = arduinoSer.readline()									#read in first line of test data
		lineReceived = lineReceived.split("/")[0]								#take first element after splitting by forward slash to strip special characters
		[loadReading, angleReading] = lineReceived.split(",")					#split load cell and potentiometer values using comma
		angleReading = float(angleReading)
		loadReading = float(loadReading)
		lineReceived = arduinoSer.readline()									#read in next line of test data
		yield angleReading, loadReading

fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_ylim(-1, 1)
ax.set_xlim(-1, 1)
ax.grid()
xdata, ydata = [], []

def run(data):
	# update the data
	angle,load = data
	xdata.append(angle)
	ydata.append(load)
	xmin, xmax = ax.get_xlim()
	ymin, ymax = ax.get_ylim()

	if angle >= xmax:
		ax.set_xlim(xmin, xmax+10)
		ax.figure.canvas.draw()
	elif angle <= xmin:
		ax.set_xlim(xmin-10, xmax)
		ax.figure.canvas.draw()
	if load >= ymax:
		ax.set_ylim(ymin, ymax+10)
		ax.figure.canvas.draw()
	elif load <= ymin:
		ax.set_ylim(ymin-10, ymax)
		ax.figure.canvas.draw()

	line.set_data(xdata, ydata)

	return line,

ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10,
    repeat=False)
plt.show()
