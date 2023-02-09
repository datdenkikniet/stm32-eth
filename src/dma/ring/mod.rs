use nonmax::NonMaxU16;

const MTU: usize = 1522;

mod buffer_ring;
pub(crate) use buffer_ring::{Buffer, BufferIndex, BufferRing};

pub trait DescriptorEntry {
    /// A function that returns [`Some`] if there is a
    /// buffer index associated with this entry that may
    /// be freed.
    fn take_buffer(&mut self) -> Option<BufferIndex>;

    /// Check if this entry has a buffer
    fn has_buffer(&self) -> bool;
}

pub trait SettableDescriptorEntry: DescriptorEntry {
    /// Set the buffer of this entry to `buffer`.
    fn set_buffer(&mut self, buffer: Buffer);
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

    pub(crate) fn entry_mut(&mut self, index: usize) -> &mut T {
        &mut self.entries[index]
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

    pub(crate) fn buffer_size(&self) -> usize {
        self.buffers.buffer_len()
    }

    pub(crate) fn next_buffer(&mut self) -> Option<Buffer> {
        self.buffers.next_buffer()
    }

    pub(crate) fn free(&mut self, index: BufferIndex) {
        self.buffers.free(index);
    }

    pub(crate) fn free_buffers(&self) -> u16 {
        self.buffers.free_buffer_count()
    }
}

impl<'data, T> EntryRing<'data, T>
where
    T: DescriptorEntry,
{
    /// Get the entry at `index` and a now-reserved buffer.
    ///
    /// Returns `None` if no buffers are available.
    pub fn entry_and_next_buffer(&mut self, index: usize) -> Option<(&mut T, Buffer)> {
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
    /// with that entry. The buffer must be freed once you are done using
    /// it.
    pub fn entry_buffer(&mut self, index: usize) -> Option<(&mut T, Buffer)> {
        let entry = &mut self.entries[index];

        let idx = entry.take_buffer()?;

        let buffer = unsafe { self.buffers.get_buffer(idx) };

        Some((entry, buffer))
    }
}

impl<'data, T> EntryRing<'data, T>
where
    T: SettableDescriptorEntry,
{
    pub fn attach_free_buffers(&mut self, start_index: usize) {
        let (after, before_inc) = self.entries.split_at_mut(start_index);
        let mut entries = before_inc.iter_mut().chain(after.iter_mut());

        while let Some(entry) = entries.next() {
            if !entry.has_buffer() {
                if let Some(next_buffer) = self.buffers.next_buffer() {
                    entry.set_buffer(next_buffer);
                } else {
                    break;
                }
            }
        }
    }
}

#[cfg(all(test, not(target_os = "none")))]
#[test]
fn entry_ring() {
    use std::vec::Vec;
    struct Test {
        done: bool,
        buffer_idx: Option<BufferIndex>,
    }

    impl DescriptorEntry for Test {
        fn take_buffer(&mut self) -> Option<BufferIndex> {
            if self.done {
                self.buffer_idx.take()
            } else {
                None
            }
        }

        fn has_buffer(&self) -> bool {
            self.buffer_idx.is_some()
        }
    }

    const BUFFERS: usize = 4;

    let mut descriptors: Vec<_> = (0..20)
        .map(|_| Test {
            done: false,
            buffer_idx: None,
        })
        .collect();

    let mut buffers = [[0u8; 1524]; BUFFERS];

    let mut entry_ring = EntryRing::new(&mut descriptors, &mut buffers);

    for entry in 0..BUFFERS - 1 {
        let (entry, next_buffer) = entry_ring.entry_and_next_buffer(entry).unwrap();
        entry.buffer_idx = Some(next_buffer.idx());
    }

    assert!(entry_ring.entry_and_next_buffer(0).is_some());

    // "Asynchronously" make the buffers finish
    for entry in 0..BUFFERS / 2 {
        entry_ring.entries[entry].done = true;
    }

    for entry in [BUFFERS - 1, 0] {
        assert!(entry_ring.entry_and_next_buffer(entry).is_some());
    }

    assert!(entry_ring.entry_and_next_buffer(1).is_none());
}
