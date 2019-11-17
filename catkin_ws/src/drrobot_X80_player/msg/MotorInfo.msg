# motor sensor data message from DrRobot Robot.

Header header    	# timestamp in the header is the time the driver
		 	# returned the battery/power reading
string robot_type	# robot type, I90 series, sentinel3 or Jaguar Power/Motion

uint32 encoder_pos	# encoder positon count
uint32 encoder_vel	# encoder velocity value
uint32 encoder_dir	# encoder direction

float32 motor_current	# motor current
uint32 motor_pwm	# output PWM value, only for Jaguar series robot
