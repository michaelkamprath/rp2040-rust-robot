use crate::system::millis::millis;
use defmt::trace;
use embedded_hal::digital::InputPin;

/// A debounced button. The button is considered pressed when the pin is equal to the level indicated by `ACTIVE`.
/// `ACTIVE` being true indicates the button is active HIGH, and false indicates active LOW. A debounce
/// period of `DEBOUNCE` milliseconds is used to prevent bouncing.
pub struct DebouncedButton<PIN: InputPin, const ACTIVE: bool, const DEBOUNCE: u32> {
    pin: PIN,
    state: bool,
    press_consumed: bool,
    last_change: u32,
}

impl<PIN: InputPin, const ACTIVE: bool, const DEBOUNCE: u32>
    DebouncedButton<PIN, ACTIVE, DEBOUNCE>
{
    /// Create a new debounced button.
    pub fn new(pin: PIN) -> Self {
        Self {
            pin,
            state: !ACTIVE,
            press_consumed: false,
            last_change: 0,
        }
    }

    /// Update the state of the button. This should be called in the main loop.
    pub fn handle_loop(&mut self) {
        // update the status of the button
        if let Ok(status) = self.pin.is_high() {
            if status != self.state {
                let change_time = millis();
                if change_time > self.last_change + DEBOUNCE {
                    trace!(
                        "button state changed from {} to {} at {}",
                        self.state,
                        status,
                        change_time
                    );
                    // the state has changed, toggle it
                    self.state = status;
                    self.last_change = change_time;

                    // if the button is pressed, set the press_consumed flag
                    if self.state == ACTIVE {
                        trace!("button pressed, setting press_consumed to false");
                        self.press_consumed = false;
                    } else {
                        self.press_consumed = true;
                    }
                }
            }
        }
    }

    /// Returns true if the button is newly and currently pressed since the last call to this function.
    pub fn is_newly_pressed(&mut self) -> bool {
        if self.state == ACTIVE && !self.press_consumed {
            self.press_consumed = true;
            true
        } else {
            false
        }
    }
}
