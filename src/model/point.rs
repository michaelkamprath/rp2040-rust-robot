#![allow(dead_code)]
use alloc::vec::Vec;
use defmt::{error, Format};
use micromath::F32Ext;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Point {
    x: i32,
    y: i32,
    forward: bool,
}

impl Point {
    pub fn new(x: i32, y: i32) -> Self {
        Point::new_with_forward(x, y, true)
    }

    pub fn new_from_string(s: &str) -> Option<Self> {
        let parts: Vec<&str> = s.split(',').collect();
        if parts.len() != 3 {
            error!("Failed to parse point from string: {}", s);
            return None;
        }
        let x = match parts[0].parse::<i32>() {
            Ok(x) => x,
            Err(_) => {
                error!("Failed to parse x coordinate from string: {}", s);
                return None;
            }
        };
        let y = match parts[1].parse::<i32>() {
            Ok(y) => y,
            Err(_) => {
                error!("Failed to parse y coordinate from string: {}", s);
                return None;
            }
        };
        let forward = match parts[2].to_lowercase().as_str() {
            "true" => true,
            "1" => true,
            "t" => true,
            "false" => false,
            "0" => false,
            "f" => false,
            _ => {
                error!("Failed to parse forward from string: {}", s);
                return None;
            }
        };

        Some(Point::new_with_forward(x, y, forward))
    }

    pub fn new_with_forward(x: i32, y: i32, forward: bool) -> Self {
        Self { x, y, forward }
    }

    pub fn x(&self) -> i32 {
        self.x
    }

    pub fn y(&self) -> i32 {
        self.y
    }

    pub fn forward(&self) -> bool {
        self.forward
    }

    /// The distance from this point to the other point
    pub fn distance_to(&self, other: &Point) -> f32 {
        let x_diff = self.x - other.x;
        let y_diff = self.y - other.y;
        ((x_diff * x_diff + y_diff * y_diff) as f32).sqrt()
    }

    /// The absolute bearing from this point to the other point, in degrees. 0 degrees pointing postively in the y axis, and
    /// the right hand rule is used to determine the angle. 90 degrees is pointing negatively in the x axis, 180 degrees
    /// radians is pointing negatively in the y axis, and -90 degrees is pointing positively in the x axis.
    /// Takes into account the forward direction of the robot.
    pub fn absolute_bearing(&self, other: &Point) -> f32 {
        let x_diff = -(other.x - self.x);
        let y_diff = other.y - self.y;
        let bearing = (x_diff as f32).atan2(y_diff as f32).to_degrees();
        let raw_bearing = if other.forward {
            bearing
        } else {
            bearing + 180.0
        };

        if raw_bearing > 180.0 {
            raw_bearing - 360.0
        } else if raw_bearing < -180.0 {
            raw_bearing + 360.0
        } else {
            raw_bearing
        }
    }
}

impl Format for Point {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "({}, {})", self.x, self.y);
    }
}

impl core::fmt::Display for Point {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}
