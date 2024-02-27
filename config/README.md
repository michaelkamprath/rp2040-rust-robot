# RP2040 Rust Robot Onboard Configuration
The SD card used on the RP2040 Rust Robot is used to store configuration, path files, and logs. The SD card must be FAT16/32 formatted and externally set up with the following directories at the root level:
* `config` - Contains the configuration files for the robot
* `paths` - Contains the path files for the robot
* `log` - Contains the log files produced by the robot


## Configuration

### `config/config.ini`
The `config.ini` file contains the configuration for the robot. The file is in the INI format and contains the following sections and key/values. All keys are optional and have default values if not present.

#### `[STRAIGHT_MOTION]`
This section contains the configuration for straight motion calculations

|          Key           | Type  | Default | Description                                                                                       |
| :--------------------: | :---: | :-----: | :------------------------------------------------------------------------------------------------ |
| `straight_left_power`  |  f32  |   1.0   | The power to apply to the left motor when moving straight. The value is a float between 0 and 1.  |
| `straight_right_power` |  f32  |   1.0   | The power to apply to the right motor when moving straight. The value is a float between 0 and 1. |
|    `straight_pid_p`    |  f32  |   0.5   | The proportional gain for the straight motion PID controller. The value is a float.               |
|    `straight_pid_i`    |  f32  |   0.0   | The integral gain for the straight motion PID controller. The value is a float.                   |
|    `straight_pid_d`    |  f32  |   0.0   | The derivative gain for the straight motion PID controller. The value is a float.                 |

#### `[TURN_MOTION]`
This section contains the configuration for turn motion calculations
|              Key              | Type  | Default | Description                                                                                                                                                                                                   |
| :---------------------------: | :---: | :-----: | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
|    `turn_left_left_power`     |  f32  |   0.5   | The power to apply to the left motor when turning left. The value is a float between 0 and 1.                                                                                                                 |
|    `turn_left_right_power`    |  f32  |   0.5   | The power to apply to the right motor when turning left. The value is a float between 0 and 1.                                                                                                                |
|    `turn_right_left_power`    |  f32  |   0.5   | The power to apply to the left motor when turning right. The value is a float between 0 and 1.                                                                                                                |
|   `turn_right_right_power`    |  f32  |   0.5   | The power to apply to the right motor when turning right. The value is a float between 0 and 1.                                                                                                               |
| `turn_left_stop_angle_delta`  |  i32  |    0    | The angle delta to stop turning left. Negative values indicate motors will be stopped before the target angle is reached, positive values indicate motors will be stopped after the target angle is reached.  |
| `turn_right_stop_angle_delta` |  i32  |    0    | The angle delta to stop turning right. Negative values indicate motors will be stopped before the target angle is reached, positive values indicate motors will be stopped after the target angle is reached. |

#### `[GENERAL]`
This section contains general configuration for the robot
|         Key          |  Type  |   Default    | Description                                                                              |
| :------------------: | :----: | :----------: | :--------------------------------------------------------------------------------------- |
| `wheel_ticks_per_mm` | float  |     3.0      | The number of encoder ticks per millimeter of wheel travel. The value is an float.       |
|    `idle_message`    | string | "Robot Idle" | The message to display on the LCD display when the robot is idle. The value is a string. |

## Paths
The `paths` directory contains the path files for the robot. The path files are in a headerless CSV format and contain the following columns:
* `x` - The x coordinate of the path point in millimeters
* `y` - The y coordinate of the path point in millimeters
* `forward` - a boolean if the robot should move forward (`true`) or backward (`false`) to this path point

Each row in the path file represents a point in the path. Points will be traversed in the order they appear in the file. Any line that begins with a hash `#` is considered a comment and will be ignored.

Path files can be created using a text editor or spreadsheet program and saved as a CSV format with the `*.pth` extension. The path files must be saved in the `paths` directory on the SD card. The robot's path selection UI on the LCD display will display the path files in the `paths` directory and allow the user to select a path to run.

## Logs
The `log` directory contains the log files produced by the robot. Each start of the robot will create a new log file using an incrementing number mainted in the `log/index.txt` file.