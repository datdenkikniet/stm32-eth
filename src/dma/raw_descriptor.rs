use volatile_register::{RO, RW};

#[cfg(all(not(feature = "stm32f1xx-hal"), feature = "f-series"))]
pub(crate) const DESC_SIZE: usize = 8;

#[cfg(feature = "stm32f1xx-hal")]
pub(crate) const DESC_SIZE: usize = 4;

#[cfg(feature = "stm32h7xx-hal")]
pub(crate) const DESC_SIZE: usize = 4;

#[repr(C)]
#[repr(align(4))]
#[derive(Clone, Copy)]
pub struct RawDescriptor {
    pub(crate) desc: [u32; DESC_SIZE],
}

impl Default for RawDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl RawDescriptor {
    pub const fn new() -> Self {
        Self {
            desc: [0; DESC_SIZE],
        }
    }

    fn r(&self, n: usize) -> &RO<u32> {
        let ro = &self.desc[n] as *const _ as *const RO<u32>;
        unsafe { &*ro }
    }

    unsafe fn rw(&mut self, n: usize) -> &mut RW<u32> {
        let rw = &mut self.desc[n] as *mut _ as *mut RW<u32>;
        &mut *rw
    }

    pub fn read(&self, n: usize) -> u32 {
        self.r(n).read()
    }

    pub unsafe fn write(&mut self, n: usize, value: u32) {
        self.rw(n).write(value)
    }

    pub unsafe fn modify<F>(&mut self, n: usize, f: F)
    where
        F: FnOnce(u32) -> u32,
    {
        self.rw(n).modify(f)
    }
}
