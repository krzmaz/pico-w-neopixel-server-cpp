//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates the possibility to send log::info/warn/error/debug! to USB serial port.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

mod binary_info;
mod panic;
mod secret;

use cyw43_pio::PioSpi;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Stack, StackResources};
use embassy_rp::dma::{AnyChannel, Channel};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_25, PIO0, PIO1, USB};
use embassy_rp::pio::{Common, FifoJoin, Instance, Pio, PioPin, ShiftConfig, ShiftDirection, StateMachine};
use embassy_rp::usb::Driver;
use embassy_rp::{bind_interrupts, clocks, into_ref, Peripheral, PeripheralRef};
use embassy_time::{Duration, Timer};
use fixed::types::U24F8;
use fixed_macro::fixed;
use heapless::Vec;
use itertools::Itertools;
use static_cell::make_static;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    PIO1_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO1>;
});

pub struct Ws2812<'d, P: Instance, const S: usize> {
    dma: PeripheralRef<'d, AnyChannel>,
    sm: StateMachine<'d, P, S>,
}

impl<'d, P: Instance, const S: usize> Ws2812<'d, P, S> {
    pub fn new(
        pio: &mut Common<'d, P>,
        mut sm: StateMachine<'d, P, S>,
        dma: impl Peripheral<P = impl Channel> + 'd,
        pin: impl PioPin,
    ) -> Self {
        into_ref!(dma);

        // Setup sm0

        // prepare the PIO program
        let side_set = pio::SideSet::new(false, 1, false);
        let mut a: pio::Assembler<32> = pio::Assembler::new_with_side_set(side_set);

        const T1: u8 = 2; // start bit
        const T2: u8 = 5; // data bit
        const T3: u8 = 3; // stop bit
        const CYCLES_PER_BIT: u32 = (T1 + T2 + T3) as u32;

        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut do_zero = a.label();
        a.set_with_side_set(pio::SetDestination::PINDIRS, 1, 0);
        a.bind(&mut wrap_target);
        // Do stop bit
        a.out_with_delay_and_side_set(pio::OutDestination::X, 1, T3 - 1, 0);
        // Do start bit
        a.jmp_with_delay_and_side_set(pio::JmpCondition::XIsZero, &mut do_zero, T1 - 1, 1);
        // Do data bit = 1
        a.jmp_with_delay_and_side_set(pio::JmpCondition::Always, &mut wrap_target, T2 - 1, 1);
        a.bind(&mut do_zero);
        // Do data bit = 0
        a.nop_with_delay_and_side_set(T2 - 1, 0);
        a.bind(&mut wrap_source);

        let prg = a.assemble_with_wrap(wrap_source, wrap_target);
        let mut cfg = embassy_rp::pio::Config::default();

        // Pin config
        let out_pin = pio.make_pio_pin(pin);
        cfg.set_out_pins(&[&out_pin]);
        cfg.set_set_pins(&[&out_pin]);

        cfg.use_program(&pio.load_program(&prg), &[&out_pin]);

        // Clock config, measured in kHz to avoid overflows
        // TODO CLOCK_FREQ should come from embassy_rp
        let clock_freq = U24F8::from_num(clocks::clk_sys_freq() / 1000);
        let ws2812_freq = fixed!(800: U24F8);
        let bit_freq = ws2812_freq * CYCLES_PER_BIT;
        cfg.clock_divider = clock_freq / bit_freq;

        // FIFO config
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            threshold: 24,
            direction: ShiftDirection::Left,
        };

        sm.set_config(&cfg);
        sm.set_enable(true);

        Self {
            dma: dma.map_into(),
            sm,
        }
    }

    pub async fn write_raw(&mut self, words: &[u32]) {
        // DMA transfer
        self.sm.tx().dma_push(self.dma.reborrow(), words).await;
    }
}

#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static, PIN_23>, PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let driver = Driver::new(p.USB, Irqs);

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO1, Irqs);
    let mut ws2812 = Ws2812::new(&mut common, sm0, p.DMA_CH1, p.PIN_2);
    spawner.spawn(logger_task(driver)).unwrap();

    let fw = include_bytes!("../embassy/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../embassy/cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio0 = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(&mut pio0.common, pio0.sm0, pio0.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    let state = make_static!(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(wifi_task(runner)));
    control.init(clm).await;
    control.set_power_management(cyw43::PowerManagementMode::None).await;
    control.gpio_set(0, true).await;

    let config = embassy_net::Config::dhcpv4(Default::default());
    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    // Init network stack
    let stack = &*make_static!(Stack::new(
        net_device,
        config,
        make_static!(StackResources::<2>::new()),
        seed
    ));

    unwrap!(spawner.spawn(net_task(stack)));

    loop {
        // use the example file to create secret.rs
        match control.join_wpa2(secret::WIFI_NETWORK, secret::WIFI_PASSWORD).await {
            Ok(_) => break,
            Err(err) => {
                log::info!("join failed with status={}", err.status);
            }
        }
    }

    // Wait for DHCP, not necessary when using static IP
    log::info!("waiting for DHCP...");
    stack.wait_config_up().await;
    log::info!("DHCP is now up! {}", stack.config_v4().unwrap().address);

    control.gpio_set(0, false).await;
    // And now we can use it!

    let mut rx_buffer = [0; 16384];
    let mut tx_buffer = [0; 1024];
    let mut read_buf = [0; 4096];
    let mut buffer = Vec::<u8, 16384>::new();
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        socket.set_keep_alive(Some(Duration::from_secs(10)));

        log::info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            log::warn!("accept error: {:?}", e);
            continue;
        }

        // info!("Received connection from {:?}", socket.remote_endpoint());

        loop {
            let count = match socket.read(&mut read_buf).await {
                Ok(0) => {
                    log::warn!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    log::warn!("read error: {:?}", e);
                    break;
                }
            };

            buffer.extend(read_buf.iter().take(count).copied());
            if buffer.len() > 1 {
                let length: u16 = u16::from_le_bytes([buffer[0], buffer[1]]);
                if buffer.len() >= (length + 2) as usize {
                    // there might be the start of the next "frame" in the input buffer
                    let overflow = buffer.len() - (length as usize + 2);
                    if overflow > 0 {
                        log::warn!("overflow len: {:?}", overflow);
                    }
                    let rgb_words: Vec<u32, 2048> = buffer
                        .iter()
                        .skip(2)
                        .take(length as usize)
                        .copied()
                        .tuples::<(_, _, _)>()
                        .map(|(r, g, b)| ((u32::from(g) << 24) | (u32::from(r) << 16) | (u32::from(b) << 8)))
                        .collect();
                    ws2812.write_raw(&rgb_words).await;
                    // let the neopixels latch on
                    Timer::after_micros(51).await;

                    buffer.clear();
                    if let Some(remainder) = read_buf.get((count - overflow)..count) {
                        // safe to unwrap since we clear the buffer above, so the remainder
                        // cannot extend its capacity
                        buffer.extend_from_slice(remainder).unwrap();
                    }
                }
            }
            // for (dest, chunk) in data.iter_mut().zip(read_buf[..n].chunks(3)) {
            //     if chunk.len() == 3 {
            //         let rgb = smart_leds::RGB {
            //             r: chunk[0],
            //             g: chunk[1],
            //             b: chunk[2],
            //         };
            //         *dest = rgb;
            //     }
            // }
            // ws2812.write(&data).await;

            // info!("rxd {}", from_utf8(&buf[..n]).unwrap());

            // match socket.write_all(&buf[..n]).await {
            //     Ok(()) => {}
            //     Err(e) => {
            //         warn!("write error: {:?}", e);
            //         break;
            //     }
            // };
            // }
            // let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
            // socket.bind(1234).unwrap();
            // // TEST CODE
            // // let mut counter = 0u8;
            // // let gray = smart_leds::RGB8::new(20, 20, 20);
            // // let black = smart_leds::RGB8::new(0, 0, 0);
            // // TEST CODE END
            // loop {
            //     let (n, _ep) = socket.recv_from(&mut buf).await.unwrap();
            //     // no logs in the tight loop outside of debugging to not mess up timing!
            //     // log::info!("rxd from {}: {:?}", _ep, buf);
            //     for (dest, chunk) in data.iter_mut().zip(buf[..n].chunks(3)) {
            //         if chunk.len() == 3 {
            //             let rgb = smart_leds::RGB {
            //                 r: chunk[0],
            //                 g: chunk[1],
            //                 b: chunk[2],
            //             };
            //             *dest = rgb;
            //         }
            //     }

            //     // TEST CODE
            //     // counter += 1;
            //     // if counter > 64 {
            //     //     counter = 0;
            //     // }

            //     // for i in 0..data.len() {
            //     //     if i <= counter as usize {
            //     //         data[i] = gray;
            //     //     } else {
            //     //         data[i] = black;
            //     //     }
            //     // }
            //     // TEST CODE END
            //     ws2812.write(&data).await;
            //     // if we want to echo what we got:
            //     // socket.send_to(&buf[..n], ep).await.unwrap();
            // }
            // }
            // TEST CODE END
            // ws2812.write(&data).await;
            // if we want to echo what we got:
            // socket.send_to(&buf[..n], ep).await.unwrap();
            // }
            // }
            // TEST CODE END
            // ws2812.write(&data).await;
            // if we want to echo what we got:
            // socket.send_to(&buf[..n], ep).await.unwrap();
        }
    }
}
