use rp2040_pac::I2C0;
use crate::drivers::{clocks::resets::Resets, gpio::gpios::{Pin, typestate::{Bidirectional, FunctionI2c, PullUp}}};

pub mod i2c0{
    use super::{I2C0, Resets, Pin, Bidirectional, FunctionI2c, PullUp};

    const TX_FIFO_SIZE: u8 = 16;

    pub struct I2c0 {
        pub i2c0: I2C0,
    }

    impl I2c0 {
        pub fn new(
            i2c0: I2C0, 
            _sda: Pin<Bidirectional<FunctionI2c<PullUp>>, 4>,
            _scl: Pin<Bidirectional<FunctionI2c<PullUp>>, 5>,
            resets: &Resets) -> Self { 
            resets.resets.reset().modify(|_, w| w.i2c0().clear_bit());

            I2c0 { i2c0 }
        }

        pub fn configure_as_master(&self) {
            self.i2c0.ic_enable().modify(|_, w| w.enable().clear_bit());//disable i2c
            // select controller mode & speed
            self.i2c0.ic_con().modify(|_, w| {
                w.speed().fast();
                w.master_mode().enabled();
                w.ic_slave_disable().slave_disabled();
                w.ic_restart_en().enabled();
                w.tx_empty_ctrl().enabled()
            });
            // Clear FIFO threshold
            self.i2c0.ic_tx_tl().write(|w| unsafe { w.tx_tl().bits(0) });
            self.i2c0.ic_rx_tl().write(|w| unsafe { w.rx_tl().bits(0) });
            
            // IC_xCNT = (ROUNDUP(MIN_SCL_HIGH_LOWtime*OSCFREQ,0))
            // IC_HCNT = (600ns * 125MHz) + 1
            // IC_LCNT = (1300ns * 125MHz) + 1
            self.i2c0.ic_fs_scl_hcnt().write(|w| unsafe { w.ic_fs_scl_hcnt().bits(76) });
            self.i2c0.ic_fs_scl_lcnt().write(|w| unsafe { w.ic_fs_scl_lcnt().bits(163) });
            // spkln = lcnt/16;
            self.i2c0.ic_fs_spklen().write(|w| unsafe {  w.ic_fs_spklen().bits(163/16) });
            // sda_tx_hold_count = freq_in [cycles/s] * 300ns for scl < 1MHz
            let sda_tx_hold_count = ((125_000_000 * 3) / 10000000) + 1;
            self.i2c0.ic_sda_hold().modify(|_r, w| unsafe { w.ic_sda_tx_hold().bits(sda_tx_hold_count as u16) });
        }

        pub fn read(&self, addr: u16, bytes: &mut [u8]) -> Result<(), u32> {
            self.i2c0.ic_enable().modify(|_, w| w.enable().clear_bit());//disable i2c
            self.i2c0.ic_tar().modify(|_, w| unsafe { w.ic_tar().bits(addr) });//slave address
            self.i2c0.ic_enable().modify(|_, w| w.enable().set_bit());//enable i2c

            let last_index = bytes.len() - 1;
            for (i, byte) in bytes.iter_mut().enumerate() {
                let first = i == 0;
                let last = i == last_index;

                // wait until there is space in the FIFO to write the next byte
                while TX_FIFO_SIZE - self.i2c0.ic_txflr().read().txflr().bits() == 0 {}

                self.i2c0.ic_data_cmd().modify(|_, w| {
                    if first {
                        w.restart().enable();
                    } else {
                        w.restart().disable();
                    }

                    if last {
                        w.stop().enable();
                    } else {
                        w.stop().disable();
                    }

                    w.cmd().read()
                }); 

                //Wait until address tx'ed
                while self.i2c0.ic_raw_intr_stat().read().tx_empty().is_inactive() {}

                while self.i2c0.ic_rxflr().read().bits() == 0 {
                    //Wait while receive FIFO empty
                    //If attempt aborts : not valid address; return error with
                    //abort reason.
                    let abort_reason = self.i2c0.ic_tx_abrt_source().read().bits();
                    //Clear ABORT interrupt
                    self.i2c0.ic_clr_tx_abrt().read();
                    if abort_reason != 0 {
                        return Err(abort_reason)
                    } 
                }

                *byte = self.i2c0.ic_data_cmd().read().dat().bits();
            }

            Ok(())
        }
        
        pub fn write(&self, addr: u16, bytes: &[u8]) -> Result<(), u32> {
            self.i2c0.ic_enable().modify(|_, w| w.enable().clear_bit());//disable i2c
            self.i2c0.ic_tar().modify(|_, w| unsafe { w.ic_tar().bits(addr) });//slave address
            self.i2c0.ic_enable().modify(|_, w| w.enable().set_bit());//enable i2c

            let last_index = bytes.len() - 1;
            for (i, byte) in bytes.iter().enumerate() {
                let last = i == last_index;

                self.i2c0.ic_data_cmd().modify(|_, w| {
                    if last {
                        w.stop().enable();
                    } else {
                        w.stop().disable();
                    }
                    unsafe { w.dat().bits(*byte)}
                }); 

                //Wait until address and data tx'ed
                while self.i2c0.ic_raw_intr_stat().read().tx_empty().is_inactive() {}

                //If attempt aborts : not valid address; return error with
                //abort reason.
                let abort_reason = self.i2c0.ic_tx_abrt_source().read().bits();
                //Clear ABORT interrupt
                self.i2c0.ic_clr_tx_abrt().read();
                if abort_reason != 0 {
                    //Wait until the STOP condition has occured
                    while self.i2c0.ic_raw_intr_stat().read().stop_det().is_inactive() {}
                    //Clear STOP interrupt
                    self.i2c0.ic_clr_stop_det().read().clr_stop_det();
                    return Err(abort_reason)
                } 

                if last {
                    //Wait until the STOP condition has occured
                    while self.i2c0.ic_raw_intr_stat().read().stop_det().is_inactive() {}
                    //Clear STOP interrupt
                    self.i2c0.ic_clr_stop_det().read().clr_stop_det();
                }
            }

            Ok(())
        }
    }
}
