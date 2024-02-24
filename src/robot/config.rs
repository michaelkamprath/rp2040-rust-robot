use alloc::string::{String, ToString};
use defmt::{debug, info, warn};
use ini_core as ini;

const WHEEL_DIAMETER: f32 = 67.0; // mm
const WHEEL_CIRCUMFERENCE: f32 = WHEEL_DIAMETER * core::f32::consts::PI;
const MOTOR_REDUCTION_RATIO: f32 = 46.8;
const ENCODER_TICKS_PER_REVOLUTION: f32 = 12.0;
const WHEEL_TICKS_PER_REVOLUTION: f32 = ENCODER_TICKS_PER_REVOLUTION * MOTOR_REDUCTION_RATIO;
const WHEEL_TICKS_PER_MM: f32 = WHEEL_TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;
const MM_PER_WHEEL_TICK: f32 = WHEEL_CIRCUMFERENCE / WHEEL_TICKS_PER_REVOLUTION;

const STRAIGHT_LEFT_MOTOR_POWER: f32 = 1.0;
const STRAIGHT_RIGHT_MOTOR_POWER: f32 = 1.0;
const STRAIGHT_PID_P: f32 = 0.5;
const STRAIGHT_PID_I: f32 = 0.0;
const STRAIGHT_PID_D: f32 = 0.0;

const DEFAULT_IDLE_MESSAGE: &str = "Robot Idle";

pub struct Config {
    // straight configuration
    pub straight_left_power: f32,
    pub straight_right_power: f32,
    pub straight_pid_p: f32,
    pub straight_pid_i: f32,
    pub straight_pid_d: f32,
    // turn configuration
    pub turn_left_left_power: f32,
    pub turn_left_right_power: f32,
    pub turn_right_left_power: f32,
    pub turn_right_right_power: f32,
    pub turn_left_stop_angle_delta: i32,
    pub turn_right_stop_angle_delta: i32,
    // general configuration
    pub wheel_ticks_per_mm: f32,
    pub idle_message: String,
}

impl Config {
    pub fn new() -> Self {
        Self {
            straight_left_power: STRAIGHT_LEFT_MOTOR_POWER,
            straight_right_power: STRAIGHT_RIGHT_MOTOR_POWER,
            straight_pid_p: STRAIGHT_PID_P,
            straight_pid_i: STRAIGHT_PID_I,
            straight_pid_d: STRAIGHT_PID_D,
            turn_left_left_power: 1.0,
            turn_left_right_power: 1.0,
            turn_right_left_power: 1.0,
            turn_right_right_power: 1.0,
            turn_left_stop_angle_delta: -12,
            turn_right_stop_angle_delta: -12,
            wheel_ticks_per_mm: WHEEL_TICKS_PER_MM,
            idle_message: String::from(DEFAULT_IDLE_MESSAGE),
        }
    }

    pub fn set_config_values(&mut self, config_str: &str) {
        let config = ini::Parser::new(config_str);
        for item in config {
            match item {
                ini::Item::Section(section) => {
                    debug!("Config section: {}", section);
                }
                ini::Item::Property(key, value) => {
                    debug!("Config key: {}, value: {}", key, value);
                    match key.trim() {
                        "straight_left_power" => {
                            if value.is_some() {
                                if let Ok(power) = value.unwrap().trim().parse::<f32>() {
                                    self.straight_left_power = power;
                                    info!("CONFIG: left motor power = {}", power);
                                }
                            }
                        }
                        "straight_right_power" => {
                            if value.is_some() {
                                if let Ok(power) = value.unwrap().trim().parse::<f32>() {
                                    self.straight_right_power = power;
                                    info!("CONFIG: right motor power = {}", power);
                                }
                            }
                        }
                        "straight_pid_p" => {
                            if value.is_some() {
                                if let Ok(pid_p) = value.unwrap().trim().parse::<f32>() {
                                    self.straight_pid_p = pid_p;
                                    info!(
                                        "CONFIG: straight PID controller parameter P = {}",
                                        pid_p
                                    );
                                }
                            }
                        }
                        "straight_pid_i" => {
                            if value.is_some() {
                                if let Ok(pid_i) = value.unwrap().trim().parse::<f32>() {
                                    self.straight_pid_i = pid_i;
                                    info!(
                                        "CONFIG: straight PID controller parameter P = {}",
                                        pid_i
                                    );
                                }
                            }
                        }
                        "straight_pid_d" => {
                            if value.is_some() {
                                if let Ok(pid_d) = value.unwrap().trim().parse::<f32>() {
                                    self.straight_pid_d = pid_d;
                                    info!(
                                        "CONFIG: straight PID controller parameter P = {}",
                                        pid_d
                                    );
                                }
                            }
                        }
                        "turn_left_left_power" => {
                            if value.is_some() {
                                if let Ok(power) = value.unwrap().trim().parse::<f32>() {
                                    self.turn_left_left_power = power;
                                    info!("CONFIG: left motor power for left turn = {}", power);
                                }
                            }
                        }
                        "turn_left_right_power" => {
                            if value.is_some() {
                                if let Ok(power) = value.unwrap().trim().parse::<f32>() {
                                    self.turn_left_right_power = power;
                                    info!("CONFIG: right motor power for left turn = {}", power);
                                }
                            }
                        }
                        "turn_right_left_power" => {
                            if value.is_some() {
                                if let Ok(power) = value.unwrap().trim().parse::<f32>() {
                                    self.turn_right_left_power = power;
                                    info!("CONFIG: left motor power for right turn = {}", power);
                                }
                            }
                        }
                        "turn_right_right_power" => {
                            if value.is_some() {
                                if let Ok(power) = value.unwrap().trim().parse::<f32>() {
                                    self.turn_right_right_power = power;
                                    info!("CONFIG: right motor power for right turn = {}", power);
                                }
                            }
                        }
                        "turn_left_stop_angle_delta" => {
                            if value.is_some() {
                                if let Ok(delta) = value.unwrap().trim().parse::<i32>() {
                                    self.turn_left_stop_angle_delta = delta;
                                    info!("CONFIG: stop angle delta for left turn = {}", delta);
                                }
                            }
                        }
                        "turn_right_stop_angle_delta" => {
                            if value.is_some() {
                                if let Ok(delta) = value.unwrap().trim().parse::<i32>() {
                                    self.turn_right_stop_angle_delta = delta;
                                    info!("CONFIG: stop angle delta for right turn = {}", delta);
                                }
                            }
                        }
                        "wheel_ticks_per_mm" => {
                            if value.is_some() {
                                if let Ok(ticks_per_mm) = value.unwrap().trim().parse::<f32>() {
                                    self.wheel_ticks_per_mm = ticks_per_mm;
                                    info!("CONFIG: wheel ticks per mm = {}", ticks_per_mm);
                                }
                            }
                        }
                        "idle_message" => {
                            if value.is_some() {
                                self.idle_message = value.unwrap().trim().to_string();
                                info!("CONFIG: idle message = {}", self.idle_message.as_str());
                            }
                        }
                        _ => {
                            warn!("CONFIG: unknown key: {}", key);
                        }
                    }
                }
                _ => {}
            }
        }
    }
}
