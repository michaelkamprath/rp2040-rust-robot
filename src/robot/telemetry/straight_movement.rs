#[derive(Copy, Clone, Default)]
pub struct StraightTelemetryRow {
    time: u32,
    left_wheel_ticks: u32,
    right_wheel_ticks: u32,
    heading: f32,
    control_signal: f32,
}

impl StraightTelemetryRow {
    pub fn header() -> &'static [&'static str; 5] {
        &["time", "left", "right", "heading", "control_signal"]
    }

    pub fn new(
        time: u32,
        left_wheel_ticks: u32,
        right_wheel_ticks: u32,
        heading: f32,
        control_signal: f32,
    ) -> Self {
        Self {
            time,
            left_wheel_ticks,
            right_wheel_ticks,
            heading,
            control_signal,
        }
    }
}

impl defmt::Format for StraightTelemetryRow {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(
            f,
            "{},{},{},{},{}",
            self.time,
            self.left_wheel_ticks,
            self.right_wheel_ticks,
            self.heading,
            self.control_signal,
        );
    }
}
