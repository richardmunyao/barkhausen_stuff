import serial
import time
import os
ser = serial.Serial("COM12", "0", timeout=0)

script_path = os.path.abspath(__file__) # i.e. /path/to/dir/try1.py
script_dir = os.path.split(script_path)[0] #i.e. /path/to/dir/
rel_path = "data.txt"
abs_file_path = os.path.join(script_dir, rel_path)
data_file = open(abs_file_path, 'a')
start_time = time.clock()

def getVoltage(buff):
	buff = buff.split(b"*")
	print(buff)
	# check if we have all 16 bits:
	for reading in buff:
		print("reading",reading)
		if len(reading) == 32:
			# split into the 4 channels:
			ch1 = reading[0:4]
			ch2 = reading[4:8]
			ch3 = reading[8:12]
			ch4 = reading[12:16]
			# convert to ints for bit checking:
			ch1 = int.from_bytes(ch1, byteorder="little", signed=False)			
			ch2 = int.from_bytes(ch2, byteorder="little", signed=False)
			ch3 = int.from_bytes(ch3, byteorder="little", signed=False)
			ch4 = int.from_bytes(ch4, byteorder="little", signed=False)

			print("CH1: ",ch1)
			# we are interested in bits 1-9 and 13-19:
			# pick the bits required, save them in result. Consider offset
			result = 0
			for x in range (1, 10):
				if (ch1 >> x & 1) == 1:
					result |= ( 1 << (x-1) )
				
			for y in range(13, 20):
				if (ch1 >> y & 1) == 1:
					result |= ( 1 << (y-4) )

			#fix for polarity:
			if (result >> 15 & 1) == 1:
				result &= ~(1<<15)  #set bit to zero
				result *= -1		#declare negative

			level = (result/0x7FFF) * 5.0
			print("level",level)			
			data_file.write( str(level) + "\n" )
			#with open(abs_file_path, 'a') as data_file:
				#data_file.write( str(level) + "\n" )
			#ser.flushInput()


while True:	
	bytesToRead = ser.inWaiting()		
	if bytesToRead >= 1200:
		buff = ser.read(bytesToRead)
		#run for one minute:
		if (time.clock() - start_time >= 10.0):
			print("end")
			break

		#getVoltage(buff)


		