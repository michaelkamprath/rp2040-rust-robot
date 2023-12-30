use core::cell::{Cell, RefCell};
use cortex_m::interrupt::Mutex;
use rp_pico::hal::{
    fugit::MicrosDurationU32,
    pac::{self, interrupt},
    timer::{Alarm, Alarm0, ScheduleAlarmError},
    Timer,
};

static MILLIS_ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));
static MILLIS_COUNT: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

pub fn init_millis(timer: &mut Timer) -> Result<(), ScheduleAlarmError> {
    cortex_m::interrupt::free(|cs| {
        let mut alarm0 = timer.alarm_0().unwrap();
        alarm0.schedule(MicrosDurationU32::Hz(1000))?;
        alarm0.enable_interrupt();
        MILLIS_ALARM.borrow(cs).replace(Some(alarm0));

        Ok(())
    })?;
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }
    Ok(())
}

pub fn millis() -> u32 {
    cortex_m::interrupt::free(|cs| MILLIS_COUNT.borrow(cs).get())
}

#[interrupt]
fn TIMER_IRQ_0() {
    cortex_m::interrupt::free(|cs| {
        if let Some(alarm) = MILLIS_ALARM.borrow(cs).borrow_mut().as_mut() {
            alarm.clear_interrupt();
            alarm.schedule(MicrosDurationU32::Hz(1000)).ok();
            MILLIS_COUNT
                .borrow(cs)
                .set(MILLIS_COUNT.borrow(cs).get() + 1);
        };
    });
}
