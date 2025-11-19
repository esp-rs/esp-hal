use super::Owner;

/// Represents operations to get bitfields of DMA descriptor flags.
pub trait DescriptorFlagFields {
    /// Creates an empty descriptor flag.
    fn empty() -> Self;

    /// Gets the specified size of the descriptor buffer.
    fn size(&self) -> usize;

    /// Sets the specified size of the descriptor buffer.
    fn set_size(&mut self, size: usize);

    /// Returns the specified number of valid bytes in the buffer that this descriptor points to.
    ///
    /// This field in a transmit descriptor is written by software and indicates how many bytes can
    /// be read from the buffer.
    ///
    /// This field in a receive descriptor is written by hardware automatically and indicates how
    /// many valid bytes have been stored into the buffer.
    fn len(&self) -> usize;

    /// Sets the specified number of valid bytes in the buffer that this descriptor points to.
    ///
    /// This field in a transmit descriptor is written by software and indicates how many bytes can
    /// be read from the buffer.
    ///
    /// This field in a receive descriptor is written by hardware automatically and indicates how
    /// many valid bytes have been stored into the buffer.
    fn set_len(&mut self, length: usize);

    /// Returns whether the flags describe an empty descriptor.
    ///
    /// # Note
    ///
    /// Returns true when the descriptor buffer contains no valid bytes.
    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Returns the `SUC_EOF` bit.
    ///
    /// For receive descriptors, software needs to clear this bit to 0, and hardware will set it to
    /// 1 after receiving data containing the EOF flag.
    /// For transmit descriptors, software needs to set this bit to 1 as needed.
    /// If software configures this bit to 1 in a descriptor, the DMA will include the EOF flag in
    /// the data sent to the corresponding peripheral, indicating to the peripheral that this
    /// data segment marks the end of one transfer phase.
    fn suc_eof(&self) -> bool;

    /// Sets the `SUC_EOF` bit.
    ///
    /// For receive descriptors, software needs to clear this bit to 0, and hardware will set it to
    /// 1 after receiving data containing the EOF flag.
    /// For transmit descriptors, software needs to set this bit to 1 as needed.
    /// If software configures this bit to 1 in a descriptor, the DMA will include the EOF flag in
    /// the data sent to the corresponding peripheral, indicating to the peripheral that this
    /// data segment marks the end of one transfer phase.
    fn set_suc_eof(&mut self, suc_eof: bool);

    /// Returns the owner.
    ///
    /// Specifies who is allowed to access the buffer that this descriptor points to.
    ///
    /// See [Owner] for details.
    fn owner(&self) -> Owner;

    /// Sets the owner.
    ///
    /// Specifies who is allowed to access the buffer that this descriptor points to.
    ///
    /// See [Owner] for details.
    fn set_owner(&mut self, owner: Owner);
}
