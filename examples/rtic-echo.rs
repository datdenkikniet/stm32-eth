#![no_std]
#![no_main]

//! For build and run instructions, see README.md
//!
//! A simple TCP echo server using RTIC.
//!
//! Starts a TCP echo server on port `1337` at `ADDRESS`. `ADDRESS` is `10.0.0.1/24` by default.

use defmt_rtt as _;
use panic_probe as _;

use smoltcp::{
    iface::{self, SocketStorage},
    wire::{self, IpAddress, Ipv4Address},
};

mod common;

const ADDRESS: (IpAddress, u16) = (IpAddress::Ipv4(Ipv4Address::new(10, 0, 0, 1)), 1337);
const MAC: [u8; 6] = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05];

#[rtic::app(device = stm32_eth::stm32, dispatchers = [SPI1])]
mod app {

    use crate::common::EthernetPhy;

    use systick_monotonic::Systick;

    use stm32_eth::{EthernetDMA, RxRingEntry, TxRingEntry};

    use smoltcp::{
        iface::{self, Interface, SocketHandle, SocketSet},
        socket::tcp::Socket as TcpSocket,
        socket::{tcp::SocketBuffer as TcpSocketBuffer, tcp::State as TcpState},
        wire::EthernetAddress,
    };

    use crate::NetworkStorage;

    #[local]
    struct Local {
        interface: Interface<'static>,
        device: &'static mut EthernetDMA<'static, 'static>,
        sockets: SocketSet<'static>,
        tcp_handle: SocketHandle,
        dhcp_handle: SocketHandle,
    }

    #[shared]
    struct Shared {}

    #[monotonic(binds = SysTick, default = true)]
    type Monotonic = Systick<1000>;

    fn now_fn() -> smoltcp::time::Instant {
        let time = monotonics::now().duration_since_epoch().ticks();
        smoltcp::time::Instant::from_millis(time as i64)
    }

    #[init(local = [
        rx_ring: [RxRingEntry; 2] = [RxRingEntry::new(),RxRingEntry::new()],
        tx_ring: [TxRingEntry; 2] = [TxRingEntry::new(),TxRingEntry::new()],
        storage: NetworkStorage = NetworkStorage::new(),
        dma: core::mem::MaybeUninit<EthernetDMA<'static, 'static>> = core::mem::MaybeUninit::uninit(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Pre-init");
        let core = cx.core;
        let p = cx.device;

        let rx_ring = cx.local.rx_ring;
        let tx_ring = cx.local.tx_ring;

        let (clocks, gpio, ethernet) = crate::common::setup_peripherals(p);
        let mono = Systick::new(core.SYST, clocks.hclk().raw());

        defmt::info!("Setting up pins");
        let (pins, mdio, mdc) = crate::common::setup_pins(gpio);

        defmt::info!("Configuring ethernet");

        let (dma, mac) = stm32_eth::new_with_mii(
            ethernet.mac,
            ethernet.mmc,
            ethernet.dma,
            rx_ring,
            tx_ring,
            clocks,
            pins,
            mdio,
            mdc,
        )
        .unwrap();

        let mut dma = cx.local.dma.write(dma);

        defmt::info!("Enabling interrupts");
        dma.enable_interrupt();

        defmt::info!("Setting up smoltcp");
        let store = cx.local.storage;

        let mut routes = smoltcp::iface::Routes::new(&mut store.routes_cache[..]);
        routes
            .add_default_ipv4_route(smoltcp::wire::Ipv4Address::UNSPECIFIED)
            .ok();

        let neighbor_cache = smoltcp::iface::NeighborCache::new(&mut store.neighbor_cache[..]);

        let rx_buffer = TcpSocketBuffer::new(&mut store.tcp_socket_storage.rx_storage[..]);
        let tx_buffer = TcpSocketBuffer::new(&mut store.tcp_socket_storage.tx_storage[..]);

        let socket = TcpSocket::new(rx_buffer, tx_buffer);

        let mut interface = iface::InterfaceBuilder::new()
            .hardware_addr(EthernetAddress::from_bytes(&crate::MAC).into())
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut store.ip_addrs[..])
            .routes(routes)
            .finalize(&mut dma);

        let mut socket_storage = SocketSet::new(&mut store.sockets[..]);

        let tcp_handle = socket_storage.add(socket);

        let dhcp_handle = socket_storage.add(smoltcp::socket::dhcpv4::Socket::new());

        let socket = socket_storage.get_mut::<TcpSocket>(tcp_handle);
        socket.listen(crate::ADDRESS).ok();

        interface
            .poll(now_fn(), &mut dma, &mut socket_storage)
            .unwrap();

        if let Ok(mut phy) = EthernetPhy::from_miim(mac, 0) {
            defmt::info!(
                "Resetting PHY as an extra step. Type: {}",
                phy.ident_string()
            );

            phy.phy_init();
        } else {
            defmt::info!("Not resetting unsupported PHY.");
        }

        defmt::info!("Setup done. Listening at {}", crate::ADDRESS);

        (
            Shared {},
            Local {
                interface,
                device: dma,
                sockets: socket_storage,
                tcp_handle,
                dhcp_handle,
            },
            init::Monotonics(mono),
        )
    }

    #[task(binds = ETH, local = [interface, tcp_handle, device, sockets, dhcp_handle, data: [u8; 512] = [0u8; 512]], priority = 2)]
    fn eth_interrupt(cx: eth_interrupt::Context) {
        let (iface, tcp_handle, dhcp_handle, buffer, device, sockets) = (
            cx.local.interface,
            cx.local.tcp_handle,
            cx.local.dhcp_handle,
            cx.local.data,
            cx.local.device,
            cx.local.sockets,
        );

        let interrupt_reason = device.interrupt_handler();
        defmt::debug!("Got an ethernet interrupt! Reason: {}", interrupt_reason);

        iface.poll(now_fn(), device, sockets).ok();

        if let Some(event) = sockets
            .get_mut::<smoltcp::socket::dhcpv4::Socket>(*dhcp_handle)
            .poll()
        {
            defmt::info!("{}", event);
        }

        let socket = sockets.get_mut::<TcpSocket>(*tcp_handle);
        if let Ok(recv_bytes) = socket.recv_slice(buffer) {
            if recv_bytes > 0 {
                socket.send_slice(&buffer[..recv_bytes]).ok();
                defmt::info!("Echoed {} bytes.", recv_bytes);
            }
        }

        if !socket.is_listening() && !socket.is_open() || socket.state() == TcpState::CloseWait {
            socket.abort();
            socket.listen(crate::ADDRESS).ok();
            defmt::warn!("Disconnected... Reopening listening socket.");
        }

        iface.poll(now_fn(), device, sockets).ok();
    }
}

/// All storage required for networking
pub struct NetworkStorage {
    pub ip_addrs: [wire::IpCidr; 1],
    pub sockets: [iface::SocketStorage<'static>; 2],
    pub tcp_socket_storage: TcpSocketStorage,
    pub neighbor_cache: [Option<(wire::IpAddress, iface::Neighbor)>; 8],
    pub routes_cache: [Option<(wire::IpCidr, iface::Route)>; 8],
}

impl NetworkStorage {
    const IP_INIT: wire::IpCidr =
        wire::IpCidr::Ipv4(wire::Ipv4Cidr::new(wire::Ipv4Address::new(10, 0, 0, 1), 24));

    pub const fn new() -> Self {
        NetworkStorage {
            ip_addrs: [Self::IP_INIT],
            neighbor_cache: [None; 8],
            routes_cache: [None; 8],
            sockets: [SocketStorage::EMPTY; 2],
            tcp_socket_storage: TcpSocketStorage::new(),
        }
    }
}

/// Storage of TCP sockets
#[derive(Copy, Clone)]
pub struct TcpSocketStorage {
    rx_storage: [u8; 512],
    tx_storage: [u8; 512],
}

impl TcpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 512],
            tx_storage: [0; 512],
        }
    }
}
