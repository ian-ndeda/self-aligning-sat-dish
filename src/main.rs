#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[entry]
fn main() -> ! {
      loop {}
}

