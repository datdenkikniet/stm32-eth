use crate::stm32::{ETHERNET_MAC, ETHERNET_PTP};

pub(crate) fn setup_ptp(_eth_mac: &ETHERNET_MAC, eth_ptp: ETHERNET_PTP) {
    _eth_mac.macimr.modify(|_, w| w.tstim().set_bit());

    eth_ptp.ptptscr.modify(|_, w| w.tse().set_bit());

    // Set sub-second increment to 20ns and initial addend to HCLK/(1/20ns) (HCLK=100MHz)
    eth_ptp.ptpssir.write(|w| unsafe { w.stssi().bits(20) });
    eth_ptp.ptptsar.write(|w| unsafe { w.tsa().bits(1 << 31) });

    eth_ptp.ptptscr.modify(|_, w| w.tsfcu().set_bit());

    eth_ptp.ptptscr.modify(|_, w| w.tsaru().set_bit());
    while eth_ptp.ptptscr.read().tsaru().bit_is_set() {}

    eth_ptp.ptptslur.write(|w| unsafe { w.bits(0) });
    eth_ptp.ptptshur.write(|w| unsafe { w.bits(0) });

    // Initialise timestamp
    eth_ptp.ptptscr.modify(|_, w| w.tssti().set_bit());
    while eth_ptp.ptptscr.read().tssti().bit_is_set() {}
}
