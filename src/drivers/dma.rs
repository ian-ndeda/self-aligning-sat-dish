use rp2040_pac::DMA;
use heapless::String;

use crate::drivers::{clocks::resets::Resets};

pub struct Dma<const INDEX: usize>;

impl<const INDEX: usize> Dma<INDEX> {
    pub fn new() -> Self { Self }

    fn set_read_addr(&self, addr: u32) {
        unsafe {
            (*DMA::ptr()).ch(INDEX)
                .ch_read_addr()
                .modify(|_, w| w.bits(addr) );
        }
    }
    
    pub fn set_write_addr(&self, addr: u32) {
        unsafe {
            (*DMA::ptr()).ch(INDEX)
                .ch_write_addr()
                .modify(|_, w| w.bits(addr) );
        }
    }

    pub fn configure(&self) -> Self {
        // Configure & enable DMA channel
        // timed by uart0 tx and data size of 32 bytes
        unsafe {
            (*DMA::ptr()).ch(INDEX)
                .ch_al1_ctrl().modify(|_, w| w
                                    // Read increment
                                    .incr_read().set_bit()
                                    // Set uart0 tx as dreq
                                    .treq_sel().bits(20)
                                    // Set data size to 1 bytes
                                    .data_size().bits(0x0)
                                    // Do  not chain
                                    .chain_to().bits(0)
                                    // Enable channel 0
                                    .en().bit(true)
                                    );
        }

        Self
    }

    fn is_busy(&self) -> bool {
        // checks if channel currently busy
        unsafe {
            (*DMA::ptr()).ch(INDEX).ch_al1_ctrl().read().busy().bit_is_set() 
        }
    }

    pub fn transfer(&self, msg: &mut String<164>) {
        self.set_read_addr(msg.as_ptr() as u32);

        // Set DMA transfer count and start
        unsafe {
            (*DMA::ptr()).ch(INDEX)
                .ch_al1_trans_count_trig().modify(|_, w| w.bits(msg.len() as u32) );
        }

        while self.is_busy() {} // busy wait while a transfer ongoing

        msg.clear();
    }
}

pub struct DmaChannels {
    pub dma0: Dma<0>,// channel 0
    //...more channels
}

impl DmaChannels {
    pub fn new(_: DMA, resets: &Resets) -> Self {
        resets.resets.reset().modify(|_, w| w.dma().clear_bit());
        while resets.resets.reset_done().read().dma().bit_is_clear() {}//Wait until dma enabled
        
        Self {
            dma0: Dma::new(),
        }
    }
}
