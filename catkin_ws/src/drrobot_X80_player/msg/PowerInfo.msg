# battery /power information message from DrRobot Robot.

Header header    	# timestamp in the header is the time the driver
		 	# returned the battery/power reading
string robot_type	# robot type, I90 series, sentinel3 or Hawk/H20 Power/Motion

#below message is only I90 series with Power control system on robot, otherwise reserved
float32 bat1_vol	# battery1 voltage
float32 bat2_vol	# battery2 voltage
float32 bat1_temp	# battery1 temperature reading, now only is the AD value
float32 bat2_temp	# battery2 temperature reading, now only is the AD value
float32 dcin_vol	# dcin power voltage reading
float32 ref_vol		# board AD reference voltage reading
uint8 power_status	# power status, referee document to get detailed info for every bit
uint8 power_path	# power selected path, please referee DrRobot document
uint8 charge_path	# charger selected path, please referee DrRobot document

