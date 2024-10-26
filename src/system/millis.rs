use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use rp_pico::hal::{timer::ScheduleAlarmError, Timer};

static MILLIS_TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));

pub fn init_millis(timer: &mut Timer) -> Result<(), ScheduleAlarmError> {
    cortex_m::interrupt::free(|cs| {
        MILLIS_TIMER.borrow(cs).replace(Some(*timer));

        Ok(())
    })?;

    Ok(())
}

/// Get the current millis count.
pub fn millis() -> u64 {
    cortex_m::interrupt::free(|cs| {
        MILLIS_TIMER
            .borrow(cs)
            .borrow()
            .as_ref()
            .unwrap()
            .get_counter()
            .ticks()
            / 1000
    })
}

pub fn millis_tenths() -> u64 {
    cortex_m::interrupt::free(|cs| {
        MILLIS_TIMER
            .borrow(cs)
            .borrow()
            .as_ref()
            .unwrap()
            .get_counter()
            .ticks()
            / 100
    })
}
