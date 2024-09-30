#[derive(Copy, Clone, Default)]
pub struct StraightTelemetryRow {
    time: u32,
    left_wheel_ticks: u32,
    right_wheel_ticks: u32,
    left_motor_power: u8,
    right_motor_power: u8,
    heading: f32,
    control_signal: f32,
}

impl StraightTelemetryRow {
    pub const DATA_COLUMNS: usize = 7;

    pub fn header() -> &'static [&'static str; StraightTelemetryRow::DATA_COLUMNS] {
        &[
            "time",
            "left_encoder",
            "right_encoder",
            "left_power",
            "right_power",
            "heading",
            "control_signal",
        ]
    }

    pub fn new(
        time: u32,
        left_wheel_ticks: u32,
        right_wheel_ticks: u32,
        left_motor_power: u8,
        right_motor_power: u8,
        heading: f32,
        control_signal: f32,
    ) -> Self {
        Self {
            time,
            left_wheel_ticks,
            right_wheel_ticks,
            left_motor_power,
            right_motor_power,
            heading,
            control_signal,
        }
    }
}

impl defmt::Format for StraightTelemetryRow {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "{},{},{},{},{},{},{}",
            self.time,
            self.left_wheel_ticks,
            self.right_wheel_ticks,
            self.left_motor_power,
            self.right_motor_power,
            self.heading,
            self.control_signal,
        );
    }
}

impl core::fmt::Display for StraightTelemetryRow {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{},{},{},{},{},{},{}",
            self.time,
            self.left_wheel_ticks,
            self.right_wheel_ticks,
            self.left_motor_power,
            self.right_motor_power,
            self.heading,
            self.control_signal,
        )
    }
}
