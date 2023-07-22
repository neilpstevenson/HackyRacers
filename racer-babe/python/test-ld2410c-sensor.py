import serial

frame_started = False
MAX_FRAME = 40

def parse_command_frame():
	global latest_command_success
	global latest_ack
	global radar_data_frame
	frame_data_length = radar_data_frame[4] + (radar_data_frame[5] << 8)
	#print(f"Command payload length {frame_data_length}")
	# Validate length
	if(frame_data_length != len(radar_data_frame)-10):
		print("Invalid length")
		return False
	latest_command_success = (radar_data_frame[8] == 0 and radar_data_frame[9] == 0)
	latest_ack = radar_data_frame[6]
	
	# Parse each command response
	# Enter config command mode
	if( frame_data_length == 8 and latest_ack == 0xff):
		print(f"Enter config: {latest_command_success}")
		return latest_command_success
	elif( frame_data_length == 4 and latest_ack == 0xfe):
		print(f"Exit config: {latest_command_success}")
		return latest_command_success
	elif( frame_data_length == 12 and latest_ack == 0xa0):
		major_minor = radar_data_frame[12:14]
		major_minor.reverse()
		bugfix = radar_data_frame[14:18]
		bugfix.reverse() 
		print(f"Firmware version: {latest_command_success} {major_minor.hex('.')}.{bugfix.hex()}")
		return latest_command_success
	# Others TO DO

def parse_data_frame():
	global latest_command_success
	global data_frame_type
	global radar_data_frame
	frame_data_length = radar_data_frame[4] + (radar_data_frame[5] << 8)
	data_frame_type = radar_data_frame[6]
	#print(f"Data payload length {frame_data_length}")
	# Validate length
	if(frame_data_length != len(radar_data_frame)-10):
		print("Invalid length")
		return False
		
	latest_command_success = (radar_data_frame[8] == 0 and radar_data_frame[9] == 0)
	latest_ack = radar_data_frame[6]
	
	# Parse each data response
	# Normal Target Data
	if( frame_data_length == 13 and radar_data_frame[6] == 0x02 and radar_data_frame[7] == 0xaa and radar_data_frame[17] == 0x55 and radar_data_frame[18] == 0x00):
		target_type = radar_data_frame[8]
		moving_target_distance = radar_data_frame[9] + (radar_data_frame[10] << 8)
		moving_target_energy = radar_data_frame[11]
		stationary_target_distance = radar_data_frame[12] + (radar_data_frame[13] << 8)
		stationary_target_energy = radar_data_frame[14]
		detection_distance = radar_data_frame[15] + (radar_data_frame[16] << 8)
		if(target_type & 0x03 == 0x00):
			print(f"no target d={detection_distance}")
		if(target_type & 0x01):
			print(f"moving at {moving_target_distance}cm e={moving_target_energy} d={detection_distance}")
		if(target_type & 0x02):
			print(f"stationary at {stationary_target_distance}cm e={stationary_target_energy} d={detection_distance}")
		return True
	# Others TO DO
	
def read_frame():
	global frame_started
	global radar_data_frame
	global ack_frame
	
	if(not frame_started):
		b=ser.read(1)
		if(len(b) == 1):
			if(b[0] == 0xf4):
				radar_data_frame = bytearray(b)
				frame_started = True
				ack_frame = False
			elif(b[0] == 0xfd):
				radar_data_frame = bytearray(b)
				frame_started = True
				ack_frame = True
	else:
		if(len(radar_data_frame) < MAX_FRAME):
			radar_data_frame.extend(ser.read(1))
			#print(radar_data_frame.hex(':'))
			if(len(radar_data_frame) > 7):
				if(radar_data_frame[0] == 0xf4 and	# Data frame received?
					radar_data_frame[1] == 0xf3 and
					radar_data_frame[2] == 0xf2 and
					radar_data_frame[3] == 0xf1 and
					radar_data_frame[-4] == 0xf8 and
					radar_data_frame[-3] == 0xf7 and
					radar_data_frame[-2] == 0xf6 and
					radar_data_frame[-1] == 0xf5):
					#print("frame" + ":".join("{:02x}".format(c) for c in radar_data_frame))
					#print(f"data frame {radar_data_frame.hex(':')}")
					frame_started = False
					if(parse_data_frame()):
						return True

					frame_started = False
					return True
				elif(radar_data_frame[0] == 0xfd and	# Command frame ack received?
					radar_data_frame[1] == 0xfc and
					radar_data_frame[2] == 0xfb and
					radar_data_frame[3] == 0xfa and
					radar_data_frame[-4] == 0x04 and
					radar_data_frame[-3] == 0x03 and
					radar_data_frame[-2] == 0x02 and
					radar_data_frame[-1] == 0x01):
					#print(f"command ack {radar_data_frame.hex(':')}")
					# Parse frame
					frame_started = False
					if(parse_command_frame()):
						return True
	return False

	
	
# Open serial port
ser = serial.Serial('/dev/ttyS0', 256000, 8, 'N', 1, timeout=0.1)
print(f"Opened {ser.name} ok")

# Configure
ser.write([0xfd, 0xfc, 0xfb, 0xfa])	# Preamble
ser.write([0x04, 0x00, 0xff, 0x00, 0x01, 0x00])	# Enter config command mode
ser.write([0x04, 0x03, 0x02, 0x01])	# Postamble

while(1):
	if(read_frame()):
		break

ser.write([0xfd, 0xfc, 0xfb, 0xfa])	# Preamble
ser.write([0x02, 0x00, 0xa0, 0x00])	# Request firmware version
ser.write([0x04, 0x03, 0x02, 0x01])	# Postamble

while(1):
	if(read_frame()):
		break

ser.write([0xfd, 0xfc, 0xfb, 0xfa])	# Preamble
ser.write([0x02, 0x00, 0xfe, 0x00])	# Exit config command mode
ser.write([0x04, 0x03, 0x02, 0x01])	# Postamble

while(1):
	if(read_frame()):
		break



# Wait for Ack

# Set to notify

# Display results
while(1):
	if(read_frame()):
		pass
	#b = ser.read(10)
	#print(":".join("{:02x}".format(c) for c in b))
