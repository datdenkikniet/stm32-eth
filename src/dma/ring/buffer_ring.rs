use nonmax::NonMaxU16;

use crate::MTU;

/// A buffer index
///
/// This uses a [`NonMaxU16`] to optimize the layout of
/// descriptors that may want to store a value of this
/// type in an `Option` if/when we switch to a [`NonMaxU32`].
///
/// [`NonMaxU32`]: nonmax::NonMaxU32
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BufferIndex(NonMaxU16);

impl BufferIndex {
    fn inc(&mut self, exclusive_upper_bound: NonMaxU16) {
        self.0 = self.incd(exclusive_upper_bound).0
    }

    fn incd(self, exclusive_upper_bound: NonMaxU16) -> Self {
        let current_val = self.0.get();
        let next_val = if current_val == exclusive_upper_bound.get() - 1 {
            // SAFETY: 0 != u16::MAX
            unsafe { NonMaxU16::new_unchecked(0) }
        } else {
            // SAFETY: current_val + 1 != upper_bound and (upper_bound + 1) <= u16::MAX - 1
            unsafe { NonMaxU16::new_unchecked(current_val + 1) }
        };

        Self(next_val)
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for BufferIndex {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "BufferIndex({})", self.0.get())
    }
}

pub struct Buffer {
    ptr: *mut u8,
    len: usize,
    buffer_idx: BufferIndex,
}

impl Buffer {
    pub fn len(&self) -> usize {
        self.len
    }

    pub fn idx(&self) -> BufferIndex {
        self.buffer_idx
    }

    /// # Safety
    ///
    /// `self` or the cloned `Self` _must_ be dropped,
    /// they may exist simultaneously but must not be used
    /// simultaneously.
    pub unsafe fn clone(&self) -> Self {
        Self {
            ptr: self.ptr,
            len: self.len,
            buffer_idx: self.buffer_idx,
        }
    }
}

impl core::ops::Deref for Buffer {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { core::slice::from_raw_parts(self.ptr as *const _, self.len) }
    }
}

impl core::ops::DerefMut for Buffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { core::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
}

pub struct BufferRing<'data> {
    // The size is MTU + 2 so that the buffer size is a multiple
    // of 4, which is required on H7s
    buffers: &'data mut [[u8; MTU + 2]],
    // The first free buffer index
    first_free: BufferIndex,
    // The last used buffer index
    last_used: BufferIndex,
    // Whether all buffers are used or not
    all_buffers_used: bool,
}

impl<'data> BufferRing<'data> {
    pub fn new(buffers: &'data mut [[u8; MTU + 2]]) -> Self {
        assert!(
            buffers.len() < u16::MAX as usize,
            "A maxiumum of {} buffers are supported.",
            u16::MAX - 1
        );
        Self {
            buffers,
            first_free: BufferIndex(
                // SAFETY: 0 != u16::MAX
                unsafe { NonMaxU16::new_unchecked(0) },
            ),
            last_used: BufferIndex(
                // SAFETY: 0 != u16::MAX
                unsafe { NonMaxU16::new_unchecked(0) },
            ),
            all_buffers_used: false,
        }
    }

    pub fn buffer_count(&self) -> NonMaxU16 {
        // SAFETY: it is not possible create a [`BufferRing`] with
        // u16::MAX buffers
        unsafe { NonMaxU16::new_unchecked(self.buffers.len() as u16) }
    }

    fn free_buffer_count(&self) -> u16 {
        let last_used = self.first_free.0.get();
        let first_free = self.last_used.0.get();

        let buf_count = self.buffer_count().get();
        // The amount of used buffers.
        //
        // This value will be 0 if all buffers are used, so
        // you must not check `used_buffers` without also using
        // `last_buffer` or `self.all_buffers_used`.
        let used_buffers = last_used.wrapping_sub(first_free) % buf_count;

        if self.all_buffers_used {
            0
        } else {
            buf_count - used_buffers
        }
    }

    pub fn next_buffer(&mut self) -> Option<Buffer> {
        let last_buffer = self.first_free.incd(self.buffer_count()) == self.last_used;

        let free_buffers = self.free_buffer_count();

        if free_buffers > 0 && !self.all_buffers_used {
            // We've now assigned the last buffer, there are none left to be
            // assigned until one is freed.
            if last_buffer {
                self.all_buffers_used = true;
            }

            let current_idx = self.first_free;

            self.first_free.inc(self.buffer_count());

            let buffer = self.buffers[current_idx.0.get() as usize];

            let buffer = Buffer {
                ptr: buffer.as_ptr() as *mut _,
                len: buffer.len(),
                buffer_idx: current_idx,
            };

            Some(buffer)
        } else {
            None
        }
    }

    pub fn free(&mut self, index: BufferIndex) {
        assert_eq!(index, self.last_used, "Non-contiguous free of buffer.");
        self.all_buffers_used = false;
        self.last_used.inc(self.buffer_count());
    }

    /// # Safety
    ///
    /// The caller must guarantee that no other [`Buffer`] whose
    /// ID is `index` exists.
    pub unsafe fn get_buffer(&mut self, index: BufferIndex) -> Buffer {
        let buffer = &self.buffers[index.0.get() as usize];
        Buffer {
            ptr: buffer.as_ptr() as *mut _,
            len: buffer.len(),
            buffer_idx: index,
        }
    }
}
