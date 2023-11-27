use core::panic::PanicInfo;

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // TODO: It seems like this only runs after the executor runs another task
    // it's something, but immediate reset on panic would be better ¯\_(ツ)_/¯
    unsafe {
        let p = embassy_rp::Peripherals::steal();
        let mut w = embassy_rp::watchdog::Watchdog::new(p.WATCHDOG);
        w.trigger_reset();
        core::hint::unreachable_unchecked();
    }
}
