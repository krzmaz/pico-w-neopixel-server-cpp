use core::panic::PanicInfo;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

/// Reset at panic, see https://github.com/rp-rs/rp-hal/issues/564
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    unsafe {
        (*pac::PSM::PTR).wdsel.write(|w| w.bits(0x0000ffff));
        (*pac::WATCHDOG::PTR).ctrl.write(|w| w.trigger().set_bit());
        core::hint::unreachable_unchecked();
    }
}
