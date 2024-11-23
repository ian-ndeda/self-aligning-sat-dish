use rp2040_pac::{XOSC, CLOCKS, PLL_SYS, RESETS, WATCHDOG};
   
pub mod resets {
    use super::RESETS;   

    pub struct Resets {
        pub resets: RESETS,
    }

    impl Resets {
        pub fn new(resets: RESETS) -> Self {
            Resets { resets }
        }
    }
}

pub mod xosc {
    use super::XOSC;

    pub struct Xosc {
        xosc: XOSC,
    }

    impl Xosc {
        pub fn new(xosc: XOSC) -> Self {
            Xosc {
                xosc,
            }
        }

        pub fn init_xosc(self) {
            self.xosc.ctrl().modify(|_, w| w.freq_range()._1_15mhz());// Set freq. range
            self.xosc.startup().write(|w| unsafe { w.delay().bits(47) });// Set the startup delay
            self.xosc.ctrl().modify(|_, w| w.enable().enable());// Enable the xosc
            while self.xosc.status().read().stable().bit_is_clear() {}// Await stabilization
        }
    }
}

pub mod clockz {
    use super::{CLOCKS, PLL_SYS, resets::Resets};

    pub struct Clocks {
        pub clocks: CLOCKS,
        pll_sys: PLL_SYS,
    }

    impl Clocks {
        pub fn new(clocks: CLOCKS, pll_sys: PLL_SYS) -> Self {
            Clocks {
                clocks,
                pll_sys,
            }
        }

        pub fn configure(self, resets: &Resets) {
            resets.resets.reset().modify(|_, w| w
                          .pll_sys().clear_bit()// deassert pll sys
                          .busctrl().clear_bit()// deassert bus ctrl??
                          );

            self.pll_sys.pwr().reset();// Turn off PLL in case it is already running
            self.pll_sys.fbdiv_int().reset();
            self.pll_sys.cs().modify(|_, w| unsafe { w.refdiv().bits(1) });// Set refdiv as 1
            self.pll_sys.fbdiv_int().write(|w| unsafe { w.bits(125) });// Set fbdiv_int as 125

            self.pll_sys.pwr().modify(|_, w| w
                               .pd().clear_bit()// Turn on PLL
                               .vcopd().clear_bit()// Turn on VCO
                               );

            while self.pll_sys.cs().read().lock().bit_is_clear() {}// Await locking of pll

            self.pll_sys.prim().modify(|_, w| unsafe {
                w.postdiv1().bits(6)// Set up postdivider 1
                    .postdiv2().bits(2)// Set up postdivider 2
            });

            self.pll_sys.pwr().modify(|_, w| w.postdivpd().clear_bit());// Turn on postdividers

            // Select ref clock source as XOSC divided by 1
            // Runs watchdog and timer
            self.clocks.clk_ref_ctrl().modify(|_, w| w.src().xosc_clksrc());
            self.clocks.clk_ref_div().modify(|_, w| unsafe { w.int().bits(1) });

            // Set pll sys as clk sys
            self.clocks.clk_sys_ctrl().modify(|_, w| w.src().clksrc_clk_sys_aux());
            self.clocks.clk_sys_div().modify(|_, w| unsafe { w.int().bits(1) });

            // Set clk sys as peripheral clock
            // Used as reference clock for Peripherals
            self.clocks.clk_peri_ctrl().modify(|_, w| w
                                        //.auxsrc().clk_sys()
                                        .enable().set_bit()
                                        );
        }
    }
}

pub mod watchdog {
    use super::WATCHDOG;

    pub struct Watchdog {
        watchdog: WATCHDOG,
    }

    impl Watchdog {
        pub fn new(watchdog: WATCHDOG) -> Self {
            Watchdog {
                watchdog
            }
        }

        pub fn start_ticks(self) {
            self.watchdog.tick().modify(|_, w| unsafe{ w
                .cycles().bits(12)// For an effective frequency of 1MHz
                .enable().set_bit()
            });
        }
    }
}

