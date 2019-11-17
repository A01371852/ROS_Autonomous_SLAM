# standard Sensor information message from DrRobot Robot.

Header header    	# timestamp in the header is the time the driver
		 	# returned the battery/power reading
string robot_type	# robot type, I90 series, sentinel3 or Hawk/H20 Power/Motion

#make sure below sensors on your robot or not
uint32[] humanSensorData		#human sensor
uint32[] tiltingSensorData    #tilting sensor X Y
uint32[] overHeatSensorData   # over heat sensor on the board
uint32 thermoSensorData	    # enviroment temperature
float32 boardPowerVol 			# control board voltage
float32 motorPowerVol			# motor power voltage
float32 servoPowerVol			# servo power voltage
float32 refVol					# AD reference voltage
float32 potVol					# potentialmeter power voltage , not used now