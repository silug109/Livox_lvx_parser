import os
import sys
import numpy
import struct
import time
# import bytearray

FILE_PATH = 'Livox LiDAR - Mid-100 Point Cloud Data %231.lvx'

if __name__ == "__main__":

	fd = open(FILE_PATH,"rb")
	# data = fd.read()

	PUBLIC_HEADER_DATA =fd.read(24)
	PUBLIC_HEADER, VERSION_A, VERSION_B, VERSION_C, VERSION_D, MAGIC_CODE = struct.unpack("=16sccccI",PUBLIC_HEADER_DATA)
	print(PUBLIC_HEADER)
	print(VERSION_A)
	print(VERSION_B)
	print(VERSION_C)
	print(VERSION_D)
	print(hex(MAGIC_CODE))

	assert int.from_bytes(VERSION_A,"little") == 1, "VERSION_A"
	assert int.from_bytes(VERSION_B,"little") == 0, "VERSION_B"
	assert int.from_bytes(VERSION_C,"little") == 0, "VERSION_C"
	assert int.from_bytes(VERSION_D,"little") == 0, "VERSION_D"
	assert hex(MAGIC_CODE) == "0xAC0EA767".lower(), "MAGIC_CODE"

	PRIVATE_HEADER_BLOCK = fd.read(1)
	DEVICE_COUNT = struct.unpack("=B", PRIVATE_HEADER_BLOCK)[0]
	print("DEVICE_COUNT:", DEVICE_COUNT)
	print()

	DEVICES_INFO = []

	for device_idx in range(DEVICE_COUNT):
		DEVICE_INFO_BLOCK = fd.read(58)

		LIDAR_SN, HUB_SN, DEVICE_IDX, DEVICE_TYPE, ROLL, PITCH, YAW, X, Y, Z = struct.unpack("=16s16sBBffffff",DEVICE_INFO_BLOCK)

		# print("LIDAR_SN ", LIDAR_SN)
		# print("HUB_SN ", HUB_SN)
		# print("DEVICE_IDX ", DEVICE_IDX)
		# print("DEVICE TYPE ", DEVICE_TYPE)
		# print("ROLL ", ROLL)
		# print("PITCH ", PITCH)
		# print("YAW ", YAW)
		# print("X ", X)
		# print("Y ", Y)
		# print("Z ", Z)

		DEVICES_INFO.append([LIDAR_SN, HUB_SN, DEVICE_IDX, ROLL, PITCH, YAW, X, Y, Z])
	print()

	FRAMES = {}
	current_timestamp = None

	POINTS = []

	start_time = time.time()

	while True:

		FRAME_HEADER_BLOCK = fd.read(32)

		if FRAME_HEADER_BLOCK == b'':
			print("end of file")
			break

		Current_offset, Next_offset, Frame_index, Package_count= struct.unpack("<qqqq", FRAME_HEADER_BLOCK)
		print(Current_offset, Next_offset, Frame_index, Package_count)

		for package_idx in range(Package_count):
			Package_block = fd.read(1319)

			device_idx, Version, Slot_id, LIDAR_id, Reserved, Status_code, Timestamp_type, Data_type, Timestamp  = struct.unpack("=BBBBBIBBQ", Package_block[:19])
			# print(device_idx, Version, Slot_id, LIDAR_id, Reserved, Status_code, Timestamp_type, Data_type, Timestamp)


			if Timestamp != current_timestamp:
				if current_timestamp:
					FRAMES[current_timestamp] = POINTS
					POINTS.clear()

					print(f"Считывание одного таймштампа заняло {time.time() - start_time}")
					start_time = time.time()
					
				current_timestamp = Timestamp

			# print(Timestamp)
			points_block = Package_block[19:]

			for idx in range(100):
				point = struct.unpack("<fffB",points_block[idx*13:(idx+1)*13])
				POINTS.append(point)

	# print(f"Всего насчитано: {len(POINTS)} points")
	fd.close()





	# for line in fd:
		# print(line)



