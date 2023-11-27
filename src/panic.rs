use core::panic::PanicInfo;

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Important to disable the interrupts, otherwise the execution might halt before resetting
    cortex_m::interrupt::disable();
    unsafe {
        let p = embassy_rp::Peripherals::steal();
        let mut w = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);
        w.trigger_reset();
        core::hint::unreachable_unchecked();
    }
}
