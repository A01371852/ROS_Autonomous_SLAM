# custom Sensor information message from DrRobot Robot.

Header header    	# timestamp in the header is the time the driver
		 	# returned the battery/power reading
string robot_type	# robot type, I90 series, sentinel3 or Hawk/H20 Power/Motion

#make sure what sensor is on your expanded sensor port
uint32[] customADData		# custom AD channel 8 channel on the board, which channel is avaiable please contact Dr Robot
uint8 customIO    #expanded IO
