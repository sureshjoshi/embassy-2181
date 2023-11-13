#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![macro_use]

use defmt::{info, *};
use embassy_executor::Spawner;
use embassy_nrf::buffered_uarte::{self, BufferedUarte, BufferedUarteRx};

use embassy_nrf::interrupt::{self, InterruptExt};

use embassy_nrf::{bind_interrupts, peripherals, uarte};


use embassy_time::{Duration, Timer};

use heapless::Vec;
use nrf_softdevice::{raw, Config, Softdevice};
use static_cell::StaticCell;
use {defmt_rtt as _, embassy_nrf as _, panic_probe as _};

#[nrf_softdevice::gatt_server]
pub struct Server {
    uart: NordicUARTService,
}

#[nrf_softdevice::gatt_service(uuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")]
pub struct NordicUARTService {
    #[characteristic(uuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E", write, write_without_response)]
    rx: Vec<u8, 256>,

    #[characteristic(uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E", notify)]
    tx: Vec<u8, 237>,
}

static SERVER: StaticCell<Server> = StaticCell::new();

static STM32_RING_BUFFER_RX: StaticCell<[u8; 256]> = StaticCell::new();
static STM32_RING_BUFFER_TX: StaticCell<[u8; 1024]> = StaticCell::new();
static STM32_BUFFERED_UARTE: StaticCell<BufferedUarte<'static, peripherals::UARTE0, peripherals::TIMER1>> =
    StaticCell::new();

bind_interrupts!(struct Irqs {
    UARTE0_UART0 => buffered_uarte::InterruptHandler<peripherals::UARTE0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello World!");

    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.gpiote_interrupt_priority = interrupt::Priority::P2;
    nrf_config.time_interrupt_priority = interrupt::Priority::P2;
    interrupt::UARTE0_UART0.set_priority(interrupt::Priority::P2);
    interrupt::UARTE1.set_priority(interrupt::Priority::P2);

    let p = embassy_nrf::init(nrf_config);

    let mut stm32_uart_config = uarte::Config::default();
    stm32_uart_config.baudrate = uarte::Baudrate::BAUD921600;
    stm32_uart_config.parity = uarte::Parity::EXCLUDED;

    let stm32_ring_buffer_rx = STM32_RING_BUFFER_RX.init_with(|| [0u8; 256]);
    let stm32_ring_buffer_tx = STM32_RING_BUFFER_TX.init_with(|| [0u8; 1024]);

    let stm32_uart = STM32_BUFFERED_UARTE.init(BufferedUarte::new_with_rtscts(
        p.UARTE0,
        p.TIMER1,
        p.PPI_CH0,
        p.PPI_CH1,
        p.PPI_GROUP0,
        Irqs,
        p.P0_29,
        p.P1_13,
        p.P0_31,
        p.P1_12,
        stm32_uart_config,
        stm32_ring_buffer_rx,
        stm32_ring_buffer_tx,
    ));

    // --- Start Softdevice Tasks ---
    let sd = initialize("Panic");
    // let server = SERVER.init(Server::new(sd).unwrap());
    unwrap!(spawner.spawn(softdevice_task(sd)));

    // --- Start UART Tasks ---
    let (stm_rx, _) = stm32_uart.split();

    unwrap!(spawner.spawn(uart_reader(stm_rx)));
    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}

pub fn initialize(name: &'static str) -> &'static mut Softdevice {
    let config = Config {
        // Took these clock settings from v1 App
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_XTAL as u8,
            rc_ctiv: 0,
            rc_temp_ctiv: 0,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_1_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 2,
            event_length: 6,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 240 }),
        conn_gatts: Some(raw::ble_gatts_conn_cfg_t {
            hvn_tx_queue_size: 32, // TODO: There seems to be suggestions to not change this directly?
        }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t { attr_tab_size: 32768 }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 1,
            central_role_count: 1,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: name.as_ptr() as *const u8 as _,
            current_len: name.len() as u16,
            max_len: name.len() as u16,
            write_perm: unsafe { core::mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(raw::BLE_GATTS_VLOC_STACK as u8),
        }),
        ..Default::default()
    };

    Softdevice::enable(&config)
}

#[embassy_executor::task]
pub async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}


#[embassy_executor::task]
pub async fn uart_reader(
    mut reader: BufferedUarteRx<'static, 'static, peripherals::UARTE0, peripherals::TIMER1>
) -> ! {
    info!("uart_reader: Starting...");
    loop {
        let buf_len = {
            let Ok(buf) = reader.fill_buf().await else {
                info!("uart_reader: Error filling buffer!");
                continue;
            };
            // for b in buf.iter() {
            //     info!("uart_reader: Received byte: {}", b);
            // }
            buf.len()
        };
        info!("Consuming {} bytes from buffer", buf_len);
        reader.consume(buf_len);
    }
    // info!("uart_reader: Done!");
}