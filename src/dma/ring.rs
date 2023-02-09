use nonmax::NonMaxU16;

const MTU: usize = 1522;

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

pub trait DescriptorEntry {
    /// A function that returns [`Some`] if there is a
    /// buffer index associated with this entry that may
    /// be freed.
    fn take_buffer(&mut self) -> Option<BufferIndex>;

    /// Check if this entry has a buffer
    fn has_buffer(&self) -> bool;
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

pub(crate) struct BufferRing<'data> {
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
    fn new(buffers: &'data mut [[u8; MTU + 2]]) -> Self {
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

    fn buffer_count(&self) -> NonMaxU16 {
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

    fn next_buffer(&mut self) -> Option<Buffer> {
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

    fn free(&mut self, index: BufferIndex) {
        assert_eq!(index, self.last_used, "Non-contiguous free of buffer.");
        self.all_buffers_used = false;
        self.last_used.inc(self.buffer_count());
    }

    /// # Safety
    ///
    /// The caller must guarantee that no other [`Buffer`] whose
    /// ID is `index` exists.
    unsafe fn get_buffer(&mut self, index: BufferIndex) -> Buffer {
        let buffer = &self.buffers[index.0.get() as usize];
        Buffer {
            ptr: buffer.as_ptr() as *mut _,
            len: buffer.len(),
            buffer_idx: index,
        }
    }
}

pub struct EntryRing<'data, T> {
    entries: &'data mut [T],
    buffers: BufferRing<'data>,
}

impl<'data, T> EntryRing<'data, T> {
    pub fn new(descriptors: &'data mut [T], buffers: &'data mut [[u8; MTU + 2]]) -> Self {
        Self {
            entries: descriptors,
            buffers: BufferRing::new(buffers),
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.entries.len()
    }

    pub(crate) fn buffer_count(&self) -> NonMaxU16 {
        self.buffers.buffer_count()
    }

    pub(crate) fn entry_count(&self) -> u16 {
        self.entries.len() as u16
    }

    /// Get the [`T`] at index `index`
    pub(crate) fn entry(&self, index: usize) -> &T {
        &self.entries[index]
    }

    pub(crate) fn entries_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.entries.iter_mut()
    }

    pub(crate) fn entries(&self) -> impl Iterator<Item = &T> {
        self.entries.iter()
    }

    pub(crate) fn buffers_and_entries<F>(&mut self, mut f: F)
    where
        F: FnMut(&mut T, Option<Buffer>),
    {
        let Self { entries, buffers } = self;

        entries.iter_mut().for_each(|e| {
            let buffer = buffers.next_buffer();
            f(e, buffer)
        });
    }

    pub(crate) fn last_entry_mut(&mut self) -> &mut T {
        &mut self.entries[self.entries.len() - 1]
    }

    pub(crate) fn entries_start_address(&self) -> *const T {
        self.entries.as_ptr()
    }
}

impl<'data, T> EntryRing<'data, T>
where
    T: DescriptorEntry,
{
    /// Get the entry at `index` and a now-reserved buffer.
    ///
    /// Returns `None` if no buffers are available.
    pub fn next_entry_and_next_buffer(&mut self, index: usize) -> Option<(&mut T, Buffer)> {
        let (after, before_inc) = self.entries.split_at_mut(index);
        let mut entries = before_inc.iter_mut().chain(after.iter_mut());

        // Free an entry whose buffer is now available.
        while let Some(entry) = entries.next() {
            if let Some(buffer) = entry.take_buffer() {
                self.buffers.free(buffer);
                break;
            }
        }

        let entry = &mut self.entries[index];
        let buffer = self.buffers.next_buffer()?;

        Some((entry, buffer))
    }

    /// Get the entry at `index`, and the buffer currently associated
    /// with that entry. The buffer must be freed or it can be used again.
    pub fn entry_buffer(&mut self, index: usize) -> Option<(&mut T, Buffer)> {
        let entry = &mut self.entries[index];

        let idx = entry.take_buffer()?;

        let buffer = unsafe { self.buffers.get_buffer(idx) };

        Some((entry, buffer))
    }
}

#[cfg(all(test, not(target_os = "none")))]
use std::collections::VecDeque;
#[cfg(all(test, not(target_os = "none")))]
#[test]
fn buffer_ring() {
    const BUFFERS: usize = 8;

    let mut descriptors = [(); 20];
    let mut buffers = [[0u8; 1524]; BUFFERS];

    let mut buf_ids = VecDeque::new();

    let mut ring = BufferRing::new(&mut buffers);

    assert_eq!(ring.free_buffer_count() as usize, BUFFERS);
    while let Some(buffer) = ring.next_buffer() {
        buf_ids.push_front(buffer);
        assert_eq!(ring.free_buffer_count() as usize, BUFFERS - buf_ids.len());
    }

    assert_eq!(ring.free_buffer_count(), 0);
    assert_eq!(buf_ids.len(), BUFFERS);

    assert_eq!(ring.free_buffer_count() as usize, 0);
    for _ in 0..4 {
        ring.free(buf_ids.pop_back().unwrap().idx());
        assert_eq!(ring.free_buffer_count() as usize, BUFFERS - buf_ids.len());
    }

    assert_eq!(ring.free_buffer_count(), 4);
    assert_eq!(buf_ids.len(), BUFFERS - 4);

    while let Some(buffer) = ring.next_buffer() {
        buf_ids.push_front(buffer);
        assert_eq!(ring.free_buffer_count() as usize, BUFFERS - buf_ids.len());
    }

    assert!(buf_ids.len() == BUFFERS);

    assert_eq!(ring.free_buffer_count() as usize, 0);
    while let Some(buffer) = buf_ids.pop_back() {
        ring.free(buffer.idx());
        assert_eq!(ring.free_buffer_count() as usize, BUFFERS - buf_ids.len());
    }
}
