//! PDM→PCM変換のPIOを使ったサンプル
#![no_std]
#![no_main]

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use bsp::entry;
use cic_fixed::CicDecimationFilter;
use cortex_m::singleton;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, PinState};
use fixed::types::I1F31;
use fugit::HertzU32;
use goertzel_algorithm::OptimizedGoertzel;
use heapless::spsc::Queue;
use panic_probe as _;
use pio_proc::pio_file;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

use bsp::hal::{
    clocks::{Clock, ClockSource, ClocksManager, InitError},
    dma::{double_buffer, DMAExt},
    gpio::FunctionPio0,
    multicore::{Multicore, Stack},
    pac,
    pio::{Buffers, PIOBuilder, PIOExt, PinDir, ShiftDirection},
    pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking},
    sio::Sio,
    watchdog::Watchdog,
    xosc::setup_xosc_blocking,
};

mod rp2040_pll_settings_for_48khz_audio;
mod vreg;

/// External high-speed crystal on the pico board is 12Mhz
const EXTERNAL_XTAL_FREQ_HZ: HertzU32 = HertzU32::from_raw(12_000_000u32);

/// RP2040の動作周波数
const RP2040_CLOCK_HZ: HertzU32 = HertzU32::from_raw(307_200_000u32);

/// PCMのサンプリング周波数
const SAMPLE_RATE: HertzU32 = HertzU32::from_raw(96_000u32);

/// PDM clockの周波数
const PDM_CLOCK_HZ: HertzU32 = HertzU32::from_raw(3_072_000);

/// PDM用PIOの動作周波数
const PDM_PIO_CLOCK_HZ: HertzU32 = HertzU32::from_raw(15_360_000);

/// PDM用PIOの分周比率の整数部分 RP2040動作周波数/PIO動作周波数
/// int + (frac/256)で分周する
const PDM_PIO_CLOCKDIV_INT: u16 = (RP2040_CLOCK_HZ.raw() / PDM_PIO_CLOCK_HZ.raw()) as u16;
/// PDM用PIOの分周比率の少数部分 Jitterを最小にするには0にするべき
const PDM_PIO_CLOCKDIV_FRAC: u8 = 0u8;

/// バッファーサイズ（サンプル）
const BUFFER_SIZE: usize = 128;
const PDM_QUEUE_SIZE: usize = BUFFER_SIZE * 4;

const CIC_DECIMATION_FACTOR: usize = (PDM_CLOCK_HZ.raw() / SAMPLE_RATE.raw()) as usize; //CICフィルターのデシメーションファクター

//CICフィルターの出力（19bit）を32bit固定小数点に正規化するためのゲイン
// const GAIN: i32 = 2i32.pow(13);
const GAIN: i32 = 2i32.pow(16); //13bitのままだと音が小さいので16bitにしてみる

/// Goertzelフィルターのブロックサイズ（サンプル）
const GOERTZEL_BLOCK_SIZE: u32 = 19200;

const GOERTZEL_NUM_TARGET_FREQUENCYS: usize = 4;
const GOERTZEL_TARGET_FREQUENCYS: [f32; GOERTZEL_NUM_TARGET_FREQUENCYS] = [
    22510f32, 22590f32, 23000f32, 23700f32, //30000f32, //30375f32, 30750f32, 31125f32,
];

enum DetectedUltrasonic {
    Case1,
    Case2,
}

impl TryFrom<u32> for DetectedUltrasonic {
    type Error = ();

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(DetectedUltrasonic::Case1),
            1 => Ok(DetectedUltrasonic::Case2),
            _ => Err(()),
        }
    }
}

// Multicore
static mut CORE1_STACK: Stack<4096> = Stack::new();

/// Core1のタスク。Core0の起動後に実行される。
fn core1_task() {
    let pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };

    let mut sio = Sio::new(pac.SIO);
    let clocks = ClocksManager::new(pac.CLOCKS);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    loop {
        if sio.fifo.is_read_ready() {
            let detected = DetectedUltrasonic::try_from(sio.fifo.read().unwrap()).unwrap();
            match detected {
                DetectedUltrasonic::Case1 => {
                    info!("Core1: Case1");
                }
                DetectedUltrasonic::Case2 => {
                    info!("Core1: Case2");
                }
            }
        }
        delay.delay_ms(100);
    }
}

#[entry]
fn main() -> ! {
    info!(
        "{=str} v{=str}",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    info!("BUFFER_SIZE: {=usize}", BUFFER_SIZE);
    info!("SAMPLE_RATE: {=u32}", SAMPLE_RATE.raw());
    info!("CIC_DECIMATION_FACTOR: {=usize}", CIC_DECIMATION_FACTOR);

    let ultrasonic_threshold = decibel_to_gain(&-70.0f32); //超音波の閾値
    info!("ultrasonic_threshold: {=f32}", ultrasonic_threshold);
    let mut ultrasonic_amplitude: [f32; GOERTZEL_NUM_TARGET_FREQUENCYS] =
        [0.0f32; GOERTZEL_NUM_TARGET_FREQUENCYS];

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    //=============================VREG===============================
    // Core電圧(vreg)を取得
    let vreg_voltage = vreg::vreg_get_voltage(&mut pac.VREG_AND_CHIP_RESET);
    info!("VREG voltage: {=u8:b}", vreg_voltage);
    // Core電圧(vreg)を1.25Vに設定
    vreg::vreg_set_voltage(&mut pac.VREG_AND_CHIP_RESET, vreg::VregVoltage::Voltage1_25);
    // Core電圧(vreg)を再度取得して確認
    let vreg_voltage = vreg::vreg_get_voltage(&mut pac.VREG_AND_CHIP_RESET);
    info!("VREG voltage: {=u8:b}", vreg_voltage);

    //=============================CLOCK===============================
    // Enable the xosc
    let xosc = setup_xosc_blocking(pac.XOSC, EXTERNAL_XTAL_FREQ_HZ)
        .map_err(InitError::XoscErr)
        .ok()
        .unwrap();

    // Start tick in watchdog
    watchdog.enable_tick_generation((EXTERNAL_XTAL_FREQ_HZ.raw() / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(pac.CLOCKS);

    // Configure PLL and clocks
    {
        // Configure PLLs
        //                   REF     FBDIV VCO            POSTDIV
        // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
        // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
        let pll_sys = setup_pll_blocking(
            pac.PLL_SYS,
            xosc.operating_frequency(),
            rp2040_pll_settings_for_48khz_audio::SYS_PLL_CONFIG_307P2MHZ,
            &mut clocks,
            &mut pac.RESETS,
        )
        .map_err(InitError::PllError)
        .ok()
        .unwrap();

        let pll_usb = setup_pll_blocking(
            pac.PLL_USB,
            xosc.operating_frequency(),
            PLL_USB_48MHZ,
            &mut clocks,
            &mut pac.RESETS,
        )
        .map_err(InitError::PllError)
        .ok()
        .unwrap();

        // Configure clocks
        // CLK_REF = XOSC (12MHz) / 1 = 12MHz
        clocks
            .reference_clock
            .configure_clock(&xosc, xosc.get_freq())
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();

        // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
        clocks
            .system_clock
            .configure_clock(&pll_sys, pll_sys.get_freq())
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();

        // CLK USB = PLL USB (48MHz) / 1 = 48MHz
        clocks
            .usb_clock
            .configure_clock(&pll_usb, pll_usb.get_freq())
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();

        // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
        clocks
            .adc_clock
            .configure_clock(&pll_usb, pll_usb.get_freq())
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();

        // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
        clocks
            .rtc_clock
            .configure_clock(&pll_usb, HertzU32::from_raw(46875u32))
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();

        // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
        // Normally choose clk_sys or clk_usb
        clocks
            .peripheral_clock
            .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
            .map_err(InitError::ClockError)
            .ok()
            .unwrap();
    }

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    //=============================GPIO===============================
    let pins = bsp::hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // LED GPIO
    let mut user_led1_pin = pins.gpio28.into_push_pull_output_in_state(PinState::Low);
    let mut user_led2_pin = pins.gpio27.into_push_pull_output_in_state(PinState::Low);

    // configure GPIO for PIO0.
    let mut pdm_power_pin = pins.gpio13.into_push_pull_output_in_state(PinState::Low);
    let pdm_input_pin = pins.gpio22.into_function::<FunctionPio0>();
    let pdm_clock_output_pin = pins.gpio26.into_function::<FunctionPio0>();
    let mut pdm_pio_jump_pin = pins.gpio16.into_push_pull_output();
    delay.delay_ms(1);
    pdm_power_pin.set_high().unwrap();
    delay.delay_ms(100); // PDMマイクのパワーアップシーケンスに50ms程度必要
    pdm_pio_jump_pin.set_high().unwrap(); //PDM用PIOの起動直後はJUMP PINをHighにしてPDM clockを1.536MHzにし、1秒くらい経ったらLowにして3.072MHzにしてUltrasonic modeにする

    //=============================PIO===============================
    let pio_pdm = pio_file!(
        "./src/pdm.pio",
        select_program("pdm_mono_SPH0641LU4H_ultrasonic_mode")
    )
    .program;

    // Initialize and start PIO
    let (mut pio, _sm0, _sm1, sm2, _sm3) = pac.PIO0.split(&mut pac.RESETS);
    let pio_pdm = pio.install(&pio_pdm).unwrap();

    // PDM用PIOの設定
    let (mut sm2, rx2, _tx1) = PIOBuilder::from_program(pio_pdm)
        .in_pin_base(pdm_input_pin.id().num)
        .side_set_pin_base(pdm_clock_output_pin.id().num)
        .jmp_pin(pdm_pio_jump_pin.id().num) // PIO起動直後はJUMP PINをHighにしてPDM clockを1.536MHzにし、1秒くらい経ったらLowにして3.072MHzにする
        .clock_divisor_fixed_point(PDM_PIO_CLOCKDIV_INT, PDM_PIO_CLOCKDIV_FRAC)
        .in_shift_direction(ShiftDirection::Left) //左シフト
        .autopush(true)
        .push_threshold(32u8) //Bit-depth: 32bit
        .buffers(Buffers::OnlyRx) // Tx FIFOは使わないので、その分をRx FIFOにjoin
        .build(sm2);

    sm2.set_pindirs([
        (pdm_input_pin.id().num, PinDir::Input),
        (pdm_clock_output_pin.id().num, PinDir::Output),
    ]);

    //=============================MULTICORE===============================
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    // core1起動
    core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, core1_task)
        .unwrap();

    //=============================DMA===============================
    let dma_channels = pac.DMA.split(&mut pac.RESETS);
    // PDM用DMA設定
    let pdm_rx_buf1 = singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(); //staticなバッファーを作る
    let pdm_rx_buf2 = singleton!(: [u32; BUFFER_SIZE] = [0; BUFFER_SIZE]).unwrap(); //staticなバッファーを作る
    let pdm_dma_config =
        double_buffer::Config::new((dma_channels.ch2, dma_channels.ch3), rx2, pdm_rx_buf1);
    let pdm_rx_transfer = pdm_dma_config.start(); //転送開始
    let mut pdm_rx_transfer = pdm_rx_transfer.write_next(pdm_rx_buf2);

    let mut l_pdm_queue: Queue<I1F31, PDM_QUEUE_SIZE> = Queue::new();
    let mut l_cic = CicDecimationFilter::<CIC_DECIMATION_FACTOR, 3>::new(); //CICフィルターの初期化
    const INPUT_BITS: u32 = 1u32; //PDMなので1bit
    let bit_growth = l_cic.bit_growth(); //CICフィルターによって増加するBit数
    let output_bits = INPUT_BITS + bit_growth; //CICフィルターから出力されるBit数
    info!("Bit growth of CIC: {}bits", bit_growth);
    info!("Output bits of CIC: {}bits", output_bits);

    //Goertzelフィルターの初期化
    let mut goertzel: [OptimizedGoertzel; GOERTZEL_NUM_TARGET_FREQUENCYS] = Default::default();
    for i in 0..GOERTZEL_NUM_TARGET_FREQUENCYS {
        goertzel[i].prepare(
            SAMPLE_RATE.raw(),
            GOERTZEL_TARGET_FREQUENCYS[i],
            GOERTZEL_BLOCK_SIZE,
        );
    }

    //PDMのPIOスタート
    sm2.start();
    delay.delay_ms(1000);
    pdm_pio_jump_pin.set_low().unwrap(); //PDM用PIOの起動直後はJUMP PINをHighにしてPDM clockを1.536MHzにし、1秒くらい経ったらLowにして3.072MHzにしてUltrasonic modeにする

    loop {
        if !l_pdm_queue.is_empty() {
            let sample = l_pdm_queue.dequeue().unwrap();
            let sample = sample.to_bits().saturating_mul(GAIN); //ゲイン調整
            let input = sample as f32 / i32::MAX as f32; //[-1.0, 1.0]
            for (i, g) in goertzel.iter_mut().enumerate() {
                if let Some(magnitude) = g.process_sample(&input) {
                    ultrasonic_amplitude[i] = magnitude;

                    if i == 0 {
                        info!("{} dB", gain_to_decibel(&magnitude));
                    }

                    if i == GOERTZEL_NUM_TARGET_FREQUENCYS - 1 {
                        // 常時鳴っている超音波の検出
                        if (ultrasonic_amplitude[0] > ultrasonic_threshold)
                            && (ultrasonic_amplitude[1] > ultrasonic_threshold)
                        {
                            if (ultrasonic_amplitude[2] > ultrasonic_threshold)
                                && (ultrasonic_amplitude[3] > ultrasonic_threshold)
                            {
                                // ケース1の周波数とケース2の周波数がどちらも検出された場合は、振幅が大きい方を優先する
                                if ultrasonic_amplitude[2] > ultrasonic_amplitude[3] {
                                    info!("case1_priority");
                                    if sio.fifo.is_write_ready() {
                                        sio.fifo.write(DetectedUltrasonic::Case1 as u32);
                                    }
                                    user_led1_pin.set_high().unwrap();
                                    user_led2_pin.set_low().unwrap();
                                } else {
                                    info!("case2_priority");
                                    if sio.fifo.is_write_ready() {
                                        sio.fifo.write(DetectedUltrasonic::Case2 as u32);
                                    }
                                    user_led1_pin.set_low().unwrap();
                                    user_led2_pin.set_high().unwrap();
                                }
                            } else if (ultrasonic_amplitude[2] > ultrasonic_threshold)
                                && (ultrasonic_amplitude[3] < ultrasonic_threshold)
                            {
                                info!("case1");
                                if sio.fifo.is_write_ready() {
                                    sio.fifo.write(DetectedUltrasonic::Case1 as u32);
                                }
                                user_led1_pin.set_high().unwrap();
                                user_led2_pin.set_low().unwrap();
                            } else if (ultrasonic_amplitude[2] < ultrasonic_threshold)
                                && (ultrasonic_amplitude[3] > ultrasonic_threshold)
                            {
                                info!("case2");
                                if sio.fifo.is_write_ready() {
                                    sio.fifo.write(DetectedUltrasonic::Case2 as u32);
                                }
                                user_led1_pin.set_low().unwrap();
                                user_led2_pin.set_high().unwrap();
                            } else {
                                //常時鳴っている周波数のみが検出された場合
                                info!("insufficient detection");
                                user_led1_pin.set_low().unwrap();
                                user_led2_pin.set_low().unwrap();
                            }
                        } else {
                            //超音波が全く検出されなかった場合
                            info!("not detected");
                            user_led1_pin.set_low().unwrap();
                            user_led2_pin.set_low().unwrap();
                        }
                    }
                }
            }
        }

        if pdm_rx_transfer.is_done() {
            let (rx_buf, next_rx_transfer) = pdm_rx_transfer.wait();

            for e in rx_buf.iter() {
                //上位ビットから順に処理する
                for i in (0..32).rev() {
                    let cic_input_value: i32 = if bit_bang(*e, i) { 1i32 } else { -1i32 }; //PDMの1bitを-1 or 1に変換
                    if let Some(v) = l_cic.process_sample(&cic_input_value) {
                        l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                    }
                }
            }

            pdm_rx_transfer = next_rx_transfer.write_next(rx_buf);
        }
    }
}

/// 32bitの値の指定したビットが1か0かを返す
/// # Arguments
/// * `v` - 32bitの値
/// * `index` - 0~31のビット位置（LSBが0）
#[inline]
const fn bit_bang(v: u32, index: u8) -> bool {
    v & (1 << index) > 0
}

/// atodb
#[allow(dead_code)]
#[inline]
fn gain_to_decibel(gain: &f32) -> f32 {
    if *gain > 0.00001f32 {
        20.0f32 * libm::log10f(*gain)
    } else {
        -100.0f32
    }
}

/// dbtoa
#[allow(dead_code)]
#[inline]
fn decibel_to_gain(decibel: &f32) -> f32 {
    if *decibel > -100.0f32 {
        libm::powf(10.0f32, *decibel * 0.05f32)
    } else {
        0.0f32
    }
}
