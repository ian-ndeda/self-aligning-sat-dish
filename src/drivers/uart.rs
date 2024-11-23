use rp2040_pac::{interrupt, UART0, UART1};
use cortex_m::peripheral::NVIC;

use crate::drivers::{clocks::resets::Resets, gpio::gpios::{Pin, typestate::{Output, Input, FunctionUart, PushPull}}};

pub mod uart0 {
    use heapless::String;
    use super::{UART0, Resets, Pin, Output, Input, FunctionUart, PushPull, interrupt, NVIC};

    pub struct Uart0 {
        pub uart0: UART0,
    }

    impl Uart0 {
        pub fn new(
            uart0: UART0,
            _tx: Pin<Output<FunctionUart<PushPull>>, 0>,
            _rx: Pin<Input<FunctionUart<PushPull>>, 1>,
            resets: &Resets) -> Self {
            resets.resets.reset().modify(|_, w| w.uart0().clear_bit());// Deassert uart0

            Uart0 { 
                uart0,
            }
        }

        pub fn configure_9600(&self) {
            // Set baudrate at 96 00
            self.uart0.uartibrd().modify(|_, w| unsafe { w.bits(813)});
            self.uart0.uartfbrd().modify(|_, w| unsafe { w.bits(51)});

            self.uart0.uartlcr_h().modify(|_, w| unsafe { w
                .fen().set_bit()// Enable FIFO
                    .wlen().bits(0b11)// Set word length as 8
            });
        }

        pub fn enable_uart0(&self) {
            self.uart0.uartcr().modify(|_, w| w
                        .uarten().set_bit()// Enable uart0
                        );
            
            self.enable_tx();
            self.enable_rx();
        }

        pub fn _disable_uart0(&self) {
            self.uart0.uartcr().modify(|_, w| w
                        .uarten().clear_bit()
                        );
            
            self._disable_tx();
            self._disable_rx();
        }
        
        fn enable_tx(&self) {
            self.uart0.uartcr().modify(|_, w| w
                        .txe().set_bit()// Enable tx
                        )
        }

        pub fn enable_rx(&self) {
            self.uart0.uartcr().modify(|_, w| w
                        .rxe().set_bit()// Enable rx
                        )
        }
        
        fn _disable_tx(&self) {
            self.uart0.uartcr().modify(|_, w| w
                        .txe().clear_bit()
                        )
        }

        pub fn _disable_rx(&self) {
            self.uart0.uartcr().modify(|_, w| w
                        .rxe().clear_bit()
                        )
        }

        pub fn enable_tx_dma(&self) {
            self.uart0.uartdmacr().modify(|_, w| w
                           .txdmae().set_bit()// Transmit DMA enable
                           )
        }

        pub fn _enable_rx_dma(&self) {
            self.uart0.uartdmacr().modify(|_, w| w
                           .rxdmae().set_bit()// Receive DMA enable.
                           )
        }

        pub fn enable_interrupt(&self) {
            self.uart0.uartimsc().modify(|_, w| w
                                       .rxim().set_bit());// set interrupt for when there data in rx fifo
        }

        pub fn _disable_interrupt(&self) {
            self.uart0.uartimsc().modify(|_, w| w
                                       .rxim().clear_bit());// unset interrupt for when data is received by uart
        }

        pub fn _unmask_interrupt(&self) {
            unsafe {
                NVIC::unmask(interrupt::UART0_IRQ);// Unmask interrupt
            }
        }

        pub fn _mask_interrupt(&self) {
            NVIC::mask(interrupt::UART0_IRQ);// mask interrupt
        }
        
        pub fn clear_interrupt(&self) {
            self.uart0.uarticr().modify(|_, w| w.rtic().bit(true));// clear rx interrupt
            //self.uart0.uarticr().modify(|_, w| w.rxic().bit(true));// clear rx int CLEARED
        }

        pub fn _transmit(&self, buffer: &mut String<164>) {
            for ch in buffer.chars() {
                //while !self.uart0.uartfr.read().txfe().bit_is_set() {}// Wait until tx holding reg empty
                self.uart0.uartdr().modify(|_, w| unsafe { w.data().bits(ch as u8)});// Send data
                //while self.uart0.uartfr().read().busy().bit_is_set()  {}// Wait until tx finished
                while self.uart0.uartfr().read().txfe().bit_is_clear()  {}// Wait until tx finished
            }

            buffer.clear()
        }

        pub fn receive_byte(&self) -> u8 {// receive 1 byte
            while self.uart0.uartfr().read().rxfe().bit_is_set() {} // wait until byte received

            //while self.uart0.uartdr().read().oe().bit_is_set() {}// wait for overrun error to clear
            //while self.uart0.uartfr.read().rxff().bit_is_set() {}// Wait until rx fifo can receive data
            
            self.uart0.uartdr().read().data().bits()// Received data
        }
    }
}

pub mod uart1 {
    use heapless::String;
    use super::{UART1, Resets, Pin, Output, Input, FunctionUart, PushPull, interrupt, NVIC};

    pub struct Uart1 {
        pub uart1: UART1,
    }

    impl Uart1 {
        pub fn new(
            uart1: UART1,
            _tx: Pin<Output<FunctionUart<PushPull>>, 8>,
            _rx: Pin<Input<FunctionUart<PushPull>>, 9>,
            resets: &Resets) -> Self {
            resets.resets.reset().modify(|_, w| w.uart1().clear_bit());// Deassert uart1

            Uart1 { 
                uart1,
            }
        }

        pub fn configure_9600(&self) {
            // Set baudrate at 96 00
            self.uart1.uartibrd().modify(|_, w| unsafe { w.bits(813)});
            self.uart1.uartfbrd().modify(|_, w| unsafe { w.bits(51)});

            self.uart1.uartlcr_h().modify(|_, w| unsafe { w
                //.fen().set_bit()// Enable FIFO
                    .wlen().bits(0b11)// Set word length as 8
            });
        }

        pub fn enable_uart1(&self) {
            self.uart1.uartcr().modify(|_, w| w
                        .uarten().set_bit()// Enable uart0
                        );
            
            self.enable_tx();
            self.enable_rx();
        }
        
        pub fn _disable_uart1(&self) {
            self.uart1.uartcr().modify(|_, w| w
                        .uarten().clear_bit()
                        );
            
            self._disable_tx();
            self._disable_rx();
        }
        

        fn enable_tx(&self) {
            self.uart1.uartcr().modify(|_, w| w
                        .txe().set_bit()// Enable tx
                        )
        }

        pub fn enable_rx(&self) {
            self.uart1.uartcr().modify(|_, w| w
                        .rxe().set_bit()// Enable rx
                        )
        }

        fn _disable_tx(&self) {
            self.uart1.uartcr().modify(|_, w| w
                        .txe().clear_bit()
                        )
        }

        pub fn _disable_rx(&self) {
            self.uart1.uartcr().modify(|_, w| w
                        .rxe().clear_bit()
                        )
        }
        
        pub fn enable_tx_dma(&self) {
            self.uart1.uartdmacr().modify(|_, w| w
                           .txdmae().set_bit()// Transmit DMA enable
                           )
        }

        pub fn _enable_rx_dma(&self) {
            self.uart1.uartdmacr().modify(|_, w| w
                           .rxdmae().set_bit()// Receive DMA enable.
                           )
        }

        pub fn enable_interrupt(&self) {
            self.uart1.uartimsc().modify(|_, w| w
                        .rtim().set_bit());// set interrupt for when there's one byte in uart rx fifo
        }

        pub fn _disable_interrupt(&self) {
            self.uart1.uartimsc().modify(|_, w| w
                                       .rxim().clear_bit());// unset interrupt for when data is received by uart
        }

        pub fn _unmask_interrupt(&self) {
            unsafe {
                NVIC::unmask(interrupt::UART1_IRQ);// Unmask interrupt
            }
        }

        pub fn _mask_interrupt(&self) {
            NVIC::mask(interrupt::UART1_IRQ);// mask interrupt
        }
        
        pub fn clear_interrupt(&self) {
            self.uart1.uarticr().modify(|_, w| w.rtic().bit(true));// clear rx interrupt
            //self.uart1.uarticr().modify(|_, w| w.rxic().bit(true));// clear rx int CLEARED
        }

        pub fn _transmit(&self, buffer: &mut String<164>) {
            for ch in buffer.chars() {
                //while !self.uart1.uartfr.read().txfe().bit_is_set() {}// Wait until tx holding reg empty
                self.uart1.uartdr().modify(|_, w| unsafe { w.data().bits(ch as u8)});// Send data
                while self.uart1.uartfr().read().busy().bit_is_set()  {}// Wait until tx finished
            }

            buffer.clear()
        }

        pub fn receive_byte(&self) -> u8 {// receive 1 byte
            while self.uart1.uartfr().read().rxfe().bit_is_set() {} // wait until byte received

            //while self.uart1.uartdr().read().oe().bit_is_set() {}// wait for overrun error to clear
            //while self.uart1.uartfr.read().rxff().bit_is_set() {}// Wait until rx fifo can receive data
            
            self.uart1.uartdr().read().data().bits()// Received data
        }
    }
}
