#![allow(non_camel_case_types, dead_code, clippy::type_complexity)]
use core::fmt::Write;

use crate::{model::point::Point, robot::Robot, system::millis::millis};
use alloc::{
    string::{String, ToString},
    vec,
    vec::Vec,
};
use defmt::{debug, info, warn};
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
    i2c::I2c,
    pwm::SetDutyCycle,
    spi::SpiDevice,
};
use micromath::F32Ext;

const PATHS_DIR: &str = "paths";
const PATH_EXTENTION: &str = "PTH";
const SELECT_PATH_TIMEOUT_MS: u64 = 3500;

pub struct Driver<
    'a,
    INA1: OutputPin,
    INA2: OutputPin,
    INB1: OutputPin,
    INB2: OutputPin,
    ENA: SetDutyCycle,
    ENB: SetDutyCycle,
    BUTT1: InputPin,
    BUTT2: InputPin,
    TWI1: I2c,
    SPI_DEV: SpiDevice<u8>,
    DELAY: DelayNs + Clone,
    LED1: OutputPin,
> {
    robot: Robot<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI1, SPI_DEV, DELAY>,
    delay: DELAY,
    led1: LED1,
    selected_path: Option<String>,
}

impl<
        'a,
        INA1: OutputPin,
        INA2: OutputPin,
        INB1: OutputPin,
        INB2: OutputPin,
        ENA: SetDutyCycle,
        ENB: SetDutyCycle,
        BUTT1: InputPin,
        BUTT2: InputPin,
        TWI1: I2c,
        SPI_DEV: SpiDevice<u8>,
        DELAY: DelayNs + Clone,
        LED1: OutputPin,
    > Driver<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI1, SPI_DEV, DELAY, LED1>
{
    pub fn new(
        robot: Robot<'a, INA1, INA2, INB1, INB2, ENA, ENB, BUTT1, BUTT2, TWI1, SPI_DEV, DELAY>,
        delay: DELAY,
        led1_pin: LED1,
    ) -> Self {
        Self {
            robot,
            delay,
            led1: led1_pin,
            selected_path: None,
        }
    }

    /// Delays for the given milliseconds, calling the robot's loop handler often. Ignores button presses.
    pub fn delay(&mut self, ms: u32) {
        // we are going to call the loop handler in the delay function every ms until the delay is complete
        let start_millis = millis();
        while millis() - start_millis < ms as u64 {
            self.robot.handle_loop();
            self.delay.delay_us(500);
        }
    }

    pub fn handle_loop(&mut self) {
        if self.robot.button1_pressed() {
            self.led1.set_high().ok();
            debug!("button1 pressed");
            if self.selected_path.is_some() {
                let filename = self.selected_path.clone();
                let path = self.load_path_file(filename.as_ref().unwrap().as_str());
                if path.is_some() {
                    info!("loaded path: {}", filename.as_ref().unwrap().as_str());
                    writeln!(
                        self.robot.logger,
                        "\n==========\nLOAD_PATH: loaded path from file {}",
                        filename.as_ref().unwrap().as_str()
                    )
                    .ok();
                    let points = path.unwrap();
                    self.trace_path(points.as_slice());
                }
            } else {
                warn!("handle_loop: no path selected");
                write!(self.robot.set_lcd_cursor(0, 1), "No path loaded!").ok();
                self.robot.start_display_reset_timer();
            }
            self.led1.set_low().ok();
        }
        if self.robot.button2_pressed() {
            debug!("button2 pressed");
            self.handle_functions_menu();
        }
        self.robot.handle_loop();
    }

    pub fn load_path_file(&mut self, path_filename: &str) -> Option<Vec<Point>> {
        if self.selected_path.is_none() {
            warn!("load_and_trace_path: no path selected");
            return None;
        }
        if self.robot.sd_card.root_dir().is_none() {
            warn!("load_and_trace_path: no sd card present");
            return None;
        }
        let root_dir = self.robot.sd_card.root_dir().unwrap();
        let paths_dir = self
            .robot
            .sd_card
            .open_directory(root_dir, PATHS_DIR)
            .unwrap();
        let mut path_file = self
            .robot
            .sd_card
            .open_file_in_dir(path_filename, paths_dir, embedded_sdmmc::Mode::ReadOnly)
            .unwrap();

        let path_size = path_file.length().ok()?;
        let mut path_buffer = vec![0; path_size as usize];
        path_file.read(&mut path_buffer).ok()?;
        let path_str = match core::str::from_utf8(&path_buffer) {
            Ok(s) => s,
            Err(e) => {
                warn!(
                    "load_and_trace_path: failed to parse path file contents: {:?}",
                    e.to_string().as_str()
                );
                return None;
            }
        };
        let mut path: Vec<Point> = Vec::new();
        debug!("Parsing path file contents:\n{}", path_str);
        for line in path_str.lines() {
            let trimmed_line = line.trim().to_string();
            if trimmed_line.starts_with('#') {
                // skip comment lines
                continue;
            }
            let point: Point = match Point::new_from_string(trimmed_line.as_str()) {
                Some(p) => p,
                None => {
                    warn!(
                        "load_and_trace_path: failed to parse point from line: {}",
                        trimmed_line.as_str()
                    );
                    continue;
                }
            };
            path.push(point);
        }
        Some(path)
    }

    pub fn trace_path(&mut self, point_sequence: &[Point]) {
        if point_sequence.len() < 2 {
            warn!("trace_path: point_sequence too short");
            return;
        }
        writeln!(
            self.robot.logger,
            "TRACE_PATH: starting trace path with {} points at {} ms",
            point_sequence.len(),
            millis(),
        )
        .ok();

        let mut cur_point = point_sequence[0];
        // start with the bearing from the first point to the second point
        let mut cur_bearing: f32 = point_sequence[0].absolute_bearing(&point_sequence[1]);

        for next_point in point_sequence[1..].iter() {
            let distance = cur_point.distance_to(next_point);
            let bearing = cur_point.absolute_bearing(next_point);
            let bearing_diff = bearing - cur_bearing;
            debug!("driving to next way point\n  cur_point: {:?}, next_point: {:?}\n  distance: {}, bearing {}, forward: {}", cur_point, next_point, distance, bearing, next_point.forward());
            writeln!(
                self.robot.logger,
                "MOVE: from way point {:?} to way point {:?}",
                cur_point, next_point
            )
            .ok();
            // turn to the correct bearing
            if bearing_diff.abs() > self.robot.min_turn_angle() {
                self.robot.turn(bearing_diff as i32);
            }

            // drive the distance
            self.robot.straight(distance as u32, next_point.forward());

            // update the current point and bearing
            cur_point = *next_point;
            cur_bearing = bearing;
        }
    }
    pub fn handle_functions_menu(&mut self) {
        // the second button has been selected while robot was idle.
        // display the functions menu on the LCD and wait for a button press to select a function
        // - button two will cycle through the functions
        // - button one will select the current function and execute it
        // - if no button is pressed for the idle wait time, return to idle state

        const FUNCTIONS: [&str; 6] = [
            "Select Path",
            "Calibrate Gyro",
            "Heading Display",
            "Drive Straight",
            "Turn Left",
            "Turn Right",
        ];
        let mut function_idx: usize = 0;
        let mut last_interaction_millis = millis();
        self.robot.clear_display_reset_timer();
        write!(
            self.robot.clear_lcd().set_lcd_cursor(0, 0),
            "Select function >"
        )
        .ok();
        write!(
            self.robot.set_lcd_cursor(0, 1),
            "{: <16}",
            FUNCTIONS[function_idx]
        )
        .ok();

        while millis() - last_interaction_millis < SELECT_PATH_TIMEOUT_MS {
            self.robot.handle_loop();
            if self.robot.button2_pressed() {
                function_idx += 1;
                function_idx %= FUNCTIONS.len();
                write!(
                    self.robot.set_lcd_cursor(0, 1),
                    "{: <16}",
                    FUNCTIONS[function_idx]
                )
                .ok();
                last_interaction_millis = millis();
            }
            if self.robot.button1_pressed() {
                match function_idx {
                    0 => {
                        let result = self.handle_function_select_path();
                        if result.is_some() {
                            self.selected_path = result;
                            self.robot
                                .set_idle_message_line2(self.selected_path.clone());
                        }
                        self.robot.start_display_reset_timer();
                    }
                    1 => {
                        write!(
                            self.robot.clear_lcd().set_lcd_cursor(0, 0),
                            "Calibrating gyro",
                        )
                        .ok();
                        self.robot.calibrate_gyro(&mut self.delay);
                        write!(self.robot.set_lcd_cursor(0, 1), "      Done      ",).ok();
                        self.robot.start_display_reset_timer();
                    }
                    2 => {
                        self.robot.display_heading().ok();
                        self.robot.set_display_to_idle();
                    }
                    3 => {
                        write!(
                            self.robot.clear_lcd().set_lcd_cursor(0, 0),
                            "Driving straight",
                        )
                        .ok();
                        self.delay.delay_ms(500);
                        self.robot.straight(1500, true);
                        self.robot.start_display_reset_timer();
                    }
                    4 => {
                        write!(
                            self.robot.clear_lcd().set_lcd_cursor(0, 0),
                            "Turning left   \x7F",
                        )
                        .ok();
                        self.delay.delay_ms(500);
                        self.robot.turn(90);
                        self.robot.start_display_reset_timer();
                    }
                    5 => {
                        write!(
                            self.robot.clear_lcd().set_lcd_cursor(0, 0),
                            "Turning right  \x7E",
                        )
                        .ok();
                        self.delay.delay_ms(500);
                        self.robot.turn(-90);
                        self.robot.start_display_reset_timer();
                    }
                    _ => {
                        warn!("handle_functions_menu: invalid function index");
                    }
                }
                return;
            }
        }
        self.robot.set_display_to_idle();
    }

    pub fn handle_function_select_path(&mut self) -> Option<String> {
        //    1. fetch all *.path from the "path" directory on sd card
        //    2. display the selection prompt on LCD line 0 and the first path on line 1
        //    3. when button 2 is pressed, display next path on line 1, circling back to the first path after the last path
        //    4. when button 1 is pressed, select the current path and make it active. Show short message on LCD line 0 and 1 about select path. let robot return to idle state after idle wait time.
        //    5. if while in the selct path state no button is pressed for the idle wait time, return to idle state without changing the active path

        // 1. fetch all *.path from the "path" directory on sd card
        if self.robot.sd_card.root_dir().is_none() {
            warn!("handle_select_path: no sd card present");
            write!(self.robot.set_lcd_cursor(0, 1), "No SD Card!").ok();
            self.robot.start_display_reset_timer();
            return None;
        }
        let root_dir = self.robot.sd_card.root_dir().unwrap();
        let paths = self
            .robot
            .sd_card
            .list_files_in_dir_with_ext(root_dir, PATHS_DIR, PATH_EXTENTION)
            .unwrap_or_default();
        if paths.is_empty() {
            write!(
                self.robot.clear_lcd().set_lcd_cursor(0, 0),
                "No paths found",
            )
            .ok();
            self.robot.start_display_reset_timer();
            return None;
        }

        // 2. display the selection prompt on LCD line 0 and the first path on line 1
        self.robot.clear_display_reset_timer();
        write!(self.robot.clear_lcd().set_lcd_cursor(0, 0), "Select path >",).ok();
        write!(
            self.robot.set_lcd_cursor(0, 1),
            "{}",
            paths[0].name.to_string().as_str()
        )
        .ok();
        let mut paths_idx: usize = 0;

        let mut last_interaction_millis = millis();
        while millis() - last_interaction_millis < SELECT_PATH_TIMEOUT_MS {
            self.robot.handle_loop();
            if self.robot.button2_pressed() {
                // 3. when button 2 is pressed, display next path on line 1, circling back to the first path after the last path
                paths_idx += 1;
                let next_path_index = paths_idx % paths.len();
                write!(
                    self.robot.set_lcd_cursor(0, 1),
                    "{: <16}",
                    paths[next_path_index].name.to_string().as_str()
                )
                .ok();
                last_interaction_millis = millis();
            }
            if self.robot.button1_pressed() {
                // 4. when button 1 is pressed, select the current path and make it active. Show short message on LCD line 0 and 1 about select path. let robot return to idle state after idle wait time.
                let selected_path_index = paths_idx % paths.len();
                write!(self.robot.clear_lcd().set_lcd_cursor(0, 0), "Path selected",).ok();
                write!(
                    self.robot.set_lcd_cursor(0, 1),
                    "{}",
                    paths[selected_path_index].name.to_string().as_str()
                )
                .ok();
                self.robot.start_display_reset_timer();
                return Some(paths[selected_path_index].name.to_string());
            }
        }
        self.robot.set_display_to_idle();
        None
    }
}
