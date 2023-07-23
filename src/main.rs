#![no_std]
#![no_main]

use core::fmt::Write;
use url_lite::{Url, ParseError};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use hal::{
    clock::{ClockControl, CpuClock},
    peripherals::Peripherals,
    peripheral::Peripheral,
    prelude::*,
    timer::TimerGroup,
    delay::Delay,
    i2c::I2C,
    dma::DmaPriority,
    gdma::Gdma,
    i2s::{DataFormat, I2s, I2s0New, I2sWriteDma, NoMclk, PinsBclkWsDout, Standard},
    Rtc,
    IO,
    Rng
};
use embedded_svc::{
    ipv4::Interface,
    wifi::{ClientConfiguration, Configuration, Wifi},
};
use esp_wifi::{
    wifi::{WifiMode, utils::create_network_interface},
    wifi_interface::WifiStack,
    current_millis, initialize, EspWifiInitFor
};
use esp_println::{print, println};
use embedded_svc::io::{Read, Write as _};
use esp_backtrace as _;
use smoltcp::{
    iface::{SocketStorage},
    wire::{Ipv4Address, DnsQueryType}
};

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

struct HRng<'a>(&'a mut Rng<'a>);

impl<'a> rand_core::CryptoRng for HRng<'a> {}
impl<'a> rand_core::RngCore for HRng<'a> {
    fn next_u32(&mut self) -> u32 {
        self.0.random()
    }

    fn next_u64(&mut self) -> u64 {
        rand_core::impls::next_u64_via_u32(self)
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.try_fill_bytes(dest).unwrap()
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.0.read(dest).map_err(|_| core::num::NonZeroU32::new(42u32).unwrap().into())
    }
}

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    println!("Starting up...");

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut delay = Delay::new(&clocks);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio9, // sda
        io.pins.gpio8, // scl
        400u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate180)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    {
        Text::with_alignment(
            "Starting...",
            display.bounding_box().center(),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Alignment::Center,
        )
        .draw(&mut display).unwrap();
        display.flush().unwrap();
    }

    let timer = timer_group1.timer0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        unsafe { Rng::new(peripherals.RNG.clone_unchecked()) },
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();
    let (wifi, _) = peripherals.RADIO.split();

    let mut socket_set_entries: [SocketStorage; 4] = Default::default();

    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiMode::Sta, &mut socket_set_entries);

    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    let mut queries: [Option<smoltcp::socket::dns::DnsQuery>; 1] = Default::default();
    wifi_stack.configure_dns(&[Ipv4Address::new(8, 8, 8, 8).into()], &mut queries);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(), password: PASSWORD.into(), ..Default::default() });
    controller.set_configuration(&client_config).unwrap();

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());
    controller.connect().unwrap();
    println!("Wait to get connected");

    {
        display.clear(BinaryColor::Off).unwrap();
        Text::with_alignment(
            "Connecting...",
            display.bounding_box().center(),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Alignment::Center,
        )
        .draw(&mut display).unwrap();
        display.flush().unwrap();
    }

    while !controller.is_connected().unwrap() { }

    while !wifi_stack.is_iface_up() { wifi_stack.work(); }

    let ip_info = wifi_stack.get_ip_info().unwrap();
    println!("got {ip_info:?}");

    {
        display.clear(BinaryColor::Off).unwrap();
        let mut s: heapless::String<16> = heapless::String::new();
        write!(s, "{:?}", ip_info.ip).unwrap();
        Text::with_alignment(
            &s,
            display.bounding_box().center(),
            MonoTextStyle::new(&FONT_6X10, BinaryColor::On),
            Alignment::Center,
        )
        .draw(&mut display).unwrap();
        display.flush().unwrap();
    }

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    /*
    let mut read_record_buffer = [0; 8192];
    let mut write_record_buffer = [0; 8192];
    let mut rng_handle = Rng::new(peripherals.RNG);
    let mut rng = HRng(&mut rng_handle);
    */

    //let url = Url::parse("http://192.168.0.120/exist.raw").unwrap();
    let url = Url::parse("http://192.168.0.120/audio-8000.raw").unwrap();
    let path = url.path.unwrap();
    let hostname = url.host.unwrap();

    let host = wifi_stack.dns_query(hostname, DnsQueryType::A).unwrap()[0];

    print!("Making HTTP request to {host} for {path}...");

    socket.work();

    socket.open(host, 80).unwrap();

    print!("Writing...");
    write!(socket, "GET {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\n\r\n",
           path, hostname).unwrap();
    socket.flush().unwrap();
    println!("OK\n");

    let dma = Gdma::new(peripherals.DMA, &mut system.peripheral_clock_control);
    let dma_channel = dma.channel0;

    let dma = dma_buffer();

    let mut tx_descriptors = [0u32; ((32000 + 4091) / 4092) * 3];
    let mut rx_descriptors = [0u32; 3];

    let i2s = I2s::new(
        peripherals.I2S0,
        NoMclk {},
        Standard::Philips,
        DataFormat::Data16Channel16,
        8000u32.Hz(),
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let i2s_tx = i2s.i2s_tx.with_pins(PinsBclkWsDout::new(
        io.pins.gpio16, io.pins.gpio18, io.pins.gpio15,
    ));

    println!("Starting audio");

    {
        let mut s = current_millis();
        let mut front_idx = 0;
        while front_idx < dma.len() {
            let len = socket.read(&mut dma[front_idx..]).unwrap();
            front_idx += len;
        }
        s = current_millis() - s;
        let speed = dma.len() as u64 * 1000 / s;
        println!("Downloaded in {s}ms => {speed}B/s");
        if speed < 128000 {
            println!("WARNING ! Wi-Fi speed is too slow to allow for audio streaming");
        }
    }

    let mut front = [0u8; 8000];
    let mut front_start = 0;
    let mut back = [0u8; 8000];
    let mut back_start = 0;

    while front_start < front.len() {
        front_start += socket.read(&mut front[front_start..]).unwrap();
    }
    front_start = 0;

    let l = dma.len();
    println!("Playing audio ({}+{} ready)", dma.len(), front.len());
    let mut transfer = i2s_tx.write_dma_circular(dma).unwrap();
    println!(">> {}", transfer.available());
    loop {
        let avail = transfer.available();
        //print!("\r{} - {avail}       ", front.len() - front_start);

        if avail > 0 {
            let avail = usize::min(avail as usize, front.len() - front_start);
            let sent = transfer.push(&front[front_start..(front_start + avail)]).unwrap();
            front_start += sent;

            if front_start >= front.len() {
                assert!(back_start == back.len()); // back is ready
                (front, back) = (back, front);
                front_start = 0;
                back_start = 0;
            }
        }

        while back_start < back.len() {
            back_start += socket.read(&mut back[back_start..]).unwrap();
        }
    }
}

fn make_http_request(wifi_stack: &WifiStack, socket: &mut esp_wifi::wifi_interface::Socket, url: Url) {
    let path = url.path.unwrap();
    let hostname = url.host.unwrap();

    let host = wifi_stack.dns_query(hostname, DnsQueryType::A).unwrap()[0];

    print!("Making HTTP request to {host} for {path}...");

    socket.work();

    socket.open(host, 80).unwrap();

    print!("Writing...");
    write!(socket, "GET {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\n\r\n",
           path, hostname).unwrap();
    socket.flush().unwrap();
    println!("OK\n");

    let wait_end = current_millis() + 20 * 1000;
    let mut buffer = [0u8; 128];
    loop {
        if let Ok(len) = socket.read(&mut buffer) {
            if let Ok(s) = core::str::from_utf8(&buffer[..len]) {
                print!("{s}");
            } else {
                print!("{:X?}", &buffer[..len]);
            }
        } else {
            break;
        }

        if current_millis() > wait_end {
            println!("\n\n\tSocket read timeout");
            break;
        }
    }
    println!();

    socket.disconnect();
}

fn make_https_request(wifi_stack: &WifiStack, socket: &mut esp_wifi::wifi_interface::Socket,
        read_record: &mut [u8], write_record: &mut [u8], rng: &mut HRng, url: Url) {
    let path = url.path.unwrap();
    let hostname = url.host.unwrap();

    let host = wifi_stack.dns_query(hostname, DnsQueryType::A).unwrap()[0];

    let config = TlsConfig::new().with_server_name(hostname);

    print!("Making HTTPS request to {host} for {path}...");

    socket.work();

    socket.open(host, 443).unwrap();

    use embedded_tls::blocking::{TlsContext, TlsConfig, TlsConnection, Aes128GcmSha256};
    use esp_wifi::wifi_interface::Socket;

    let mut tls: TlsConnection<&mut Socket, Aes128GcmSha256> = TlsConnection::new(
        socket,
        read_record,
        write_record,
    );

    print!("Handshake...");
    tls.open::<HRng, embedded_tls::blocking::NoVerify>(TlsContext::new(
        &config, rng,
    )).expect("error establishing TLS connection");

    print!("Writing...");
    write!(tls, "GET {} HTTP/1.1\r\nHost: {}\r\nConnection: close\r\n\r\n",
           path, hostname).unwrap();
    tls.flush().unwrap();
    println!("OK\n");

    let wait_end = current_millis() + 20 * 1000;
    let mut buffer = [0u8; 128];
    loop {
        if let Ok(len) = tls.read(&mut buffer) {
            if let Ok(s) = core::str::from_utf8(&buffer[..len]) {
                print!("{s}");
            } else {
                print!("{:X?}", &buffer[..len]);
            }
        } else {
            break;
        }

        if current_millis() > wait_end {
            println!("\n\n\tSocket read timeout");
            break;
        }
    }
    println!();

    tls.close().ok();
    socket.disconnect();
}

/*
fn play_audio<'d, T, P, CH, BUFFER>(wifi_stack: &WifiStack, socket: &mut esp_wifi::wifi_interface::Socket, i2s: peripherals::I2S, dma_channel: , url: Url) {
}
*/

fn dma_buffer() -> &'static mut [u8; 32000] {
    // one second of storage at 8Khz sampling rate (16 bits 2 channels)
    // 16 * 2 / 8 * 8000 = 32 Kb/s 
    static mut BUFFER: [u8; 32000] = [0u8; 32000];
    unsafe { &mut BUFFER }
}
