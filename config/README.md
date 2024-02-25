# RP2040 Rust Robot Onboard Configuration
The SD card used on the RP2040 Rust Robot is used to store configuration, path files, and logs. The SD card must be externally set up with the following directories at the root level:
* `config` - Contains the configuration files for the robot
* `log` - Contains the log files produced by the robot
* `paths` - Contains the path files for the robot

## Configuration

### `config/config.ini`
The `config.ini` file contains the configuration for the robot. The file is in the INI format and contains the following sections and key/values:
* `STRAIGHT_MOTION` - Contains the configuration for straight motion calculations
  * `straight_left_power` - The power to apply to the left motor when moving straight. The value is a float between 0 and 1.
  * `straight_right_power` - The power to apply to the right motor when moving straight. The value is a float between 0 and 1.
  * `straight_pid_p` - The proportional gain for the straight motion PID controller. The value is a float.
  * `straight_pid_i` - The integral gain for the straight motion PID controller. The value is a float.
  * `straight_pid_d` - The derivative gain for the straight motion PID controller. The value is a float.
* `TURN_MOTION` - Contains the configuration for turn motion calculations
  * `turn_left_left_power` - The power to apply to the left motor when turning left. The value is a float between 0 and 1.
  * `turn_left_right_power` - The power to apply to the right motor when turning left. The value is a float between 0 and 1.
  * `turn_right_left_power` - The power to apply to the left motor when turning right. The value is a float between 0 and 1.
  * `turn_right_right_power` - The power to apply to the right motor when turning right. The value is a float between 0 and 1.
  * `turn_left_stop_angle_delta` - The angle delta to stop turning left. The value is a float. Negative values indicate motors will be stopped before the target angle is reached, positive values indicate motors will be stopped after the target angle is reached.
  * `turn_right_stop_angle_delta` - The angle delta to stop turning right. The value is a float. Negative values indicate motors will be stopped before the target angle is reached, positive values indicate motors will be stopped after the target angle is reached.
* `GENERAL` - Contains general configuration for the robot
  * `wheel_ticks_per_mm` - The number of encoder ticks per millimeter of wheel travel. The value is an float.
  * `idle_message` - The message to display on the LCD display when the robot is idle. The value is a string. 
