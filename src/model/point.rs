#![allow(dead_code)]
use defmt::Format;
use micromath::F32Ext;

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Point {
    x: i32,
    y: i32,
}

impl Point {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    pub fn x(&self) -> i32 {
        self.x
    }

    pub fn y(&self) -> i32 {
        self.y
    }

    /// The distance from this point to the other point
    pub fn distance_to(&self, other: &Point) -> f32 {
        let x_diff = self.x - other.x;
        let y_diff = self.y - other.y;
        ((x_diff * x_diff + y_diff * y_diff) as f32).sqrt()
    }

    /// The absolute bearing from this point to the other point, in degrees. 0 degrees pointing postively in the y axis, and
    /// the right hand rule is used to determine the angle. 90 degrees is pointing negatively in the x axis, 180 degrees
    /// radians is pointing negatively in the y axis, and -90 degrees is pointing positively in the x axis
    pub fn absolute_bearing(&self, other: &Point) -> f32 {
        let x_diff = -(other.x - self.x);
        let y_diff = other.y - self.y;
        (x_diff as f32).atan2(y_diff as f32).to_degrees()
    }
}

impl Format for Point {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "({}, {})", self.x, self.y);
    }
}