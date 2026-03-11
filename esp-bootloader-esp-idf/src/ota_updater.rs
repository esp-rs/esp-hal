//! # A more convenient way to access Over The Air Updates (OTA) functionality.

use crate::{
    ota::OtaImageState,
    partitions::{AppPartitionSubType, Error, FlashRegion, PartitionTable},
};

/// This can be used as more convenient - yet less flexible, way to do OTA updates.
///
/// If you need lower level access see [crate::ota::Ota]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OtaUpdater<'a, F>
where
    F: embedded_storage::Storage,
{
    flash: &'a mut F,
    pt: PartitionTable<'a>,
    ota_count: usize,
}

impl<'a, F> OtaUpdater<'a, F>
where
    F: embedded_storage::Storage,
{
    /// Create a new instance of [OtaUpdater].
    ///
    /// # Errors
    /// [Error::Invalid] if no OTA data partition or less than two OTA app partition were found.
    pub fn new(
        flash: &'a mut F,
        buffer: &'a mut [u8; crate::partitions::PARTITION_TABLE_MAX_LEN],
    ) -> Result<Self, Error> {
        let pt = crate::partitions::read_partition_table(flash, buffer)?;

        let mut ota_count = 0;
        let mut has_ota_data = false;
        for part in pt.iter() {
            match part.partition_type() {
                crate::partitions::PartitionType::App(subtype)
                    if subtype != crate::partitions::AppPartitionSubType::Factory
                        && subtype != crate::partitions::AppPartitionSubType::Test =>
                {
                    ota_count += 1;
                }
                crate::partitions::PartitionType::Data(
                    crate::partitions::DataPartitionSubType::Ota,
                ) => {
                    has_ota_data = true;
                }
                _ => {}
            }
        }

        if !has_ota_data {
            return Err(Error::Invalid);
        }

        if ota_count < 2 {
            return Err(Error::Invalid);
        }

        Ok(Self {
            flash,
            pt,
            ota_count,
        })
    }

    /// Returns an [`Ota`] for accessing the OTA-data partition.
    ///
    /// # Errors
    /// [Error::Invalid] if no OTA data partition was found.
    pub fn ota_data(&mut self) -> Result<crate::ota::Ota<'_, F>, Error> {
        let ota_part = self
            .pt
            .find_partition(crate::partitions::PartitionType::Data(
                crate::partitions::DataPartitionSubType::Ota,
            ))?;
        if let Some(ota_part) = ota_part {
            let ota_part = ota_part.as_embedded_storage(self.flash);
            let ota = crate::ota::Ota::new(ota_part, self.ota_count)?;
            Ok(ota)
        } else {
            Err(Error::Invalid)
        }
    }

    fn next_ota_part(&mut self) -> Result<crate::partitions::AppPartitionSubType, Error> {
        let current = self.selected_partition()?;
        let next = match current {
            AppPartitionSubType::Factory => AppPartitionSubType::Ota0,
            _ => AppPartitionSubType::from_ota_app_number(
                (current.ota_app_number() + 1) % self.ota_count as u8,
            )?,
        };

        // make sure we don't select the currently booted partition
        let booted = self.pt.booted_partition()?;
        let next = if let Some(booted) = booted {
            if booted.partition_type() == crate::partitions::PartitionType::App(next) {
                AppPartitionSubType::from_ota_app_number(
                    (current.ota_app_number() + 2) % self.ota_count as u8,
                )?
            } else {
                next
            }
        } else {
            next
        };

        Ok(next)
    }

    /// Returns the currently selected app partition.
    pub fn selected_partition(&mut self) -> Result<crate::partitions::AppPartitionSubType, Error> {
        self.ota_data()?.current_app_partition()
    }

    /// Get the [OtaImageState] of the currently selected partition.
    ///
    /// # Errors
    /// A [Error::InvalidState] if no partition is currently selected.
    pub fn current_ota_state(&mut self) -> Result<OtaImageState, Error> {
        self.ota_data()?.current_ota_state()
    }

    /// Set the [OtaImageState] of the currently selected slot.
    ///
    /// # Errors
    /// A [Error::InvalidState] if no partition is currently selected.
    pub fn set_current_ota_state(&mut self, state: OtaImageState) -> Result<(), Error> {
        self.ota_data()?.set_current_ota_state(state)
    }

    /// Selects the next active OTA-slot as current.
    ///
    /// After calling this other functions referencing the current partition will use the newly
    /// activated partition.
    pub fn activate_next_partition(&mut self) -> Result<(), Error> {
        let next_slot = self.next_ota_part()?;
        self.ota_data()?.set_current_app_partition(next_slot)
    }

    /// Returns a [FlashRegion] along with the [AppPartitionSubType] for the
    /// partition which would be selected by [Self::activate_next_partition].
    pub fn next_partition(&mut self) -> Result<(FlashRegion<'_, F>, AppPartitionSubType), Error> {
        let next_slot = self.next_ota_part()?;

        let flash_region = self
            .pt
            .find_partition(crate::partitions::PartitionType::App(next_slot))?
            .ok_or(Error::Invalid)?
            .as_embedded_storage(self.flash);

        Ok((flash_region, next_slot))
    }
}
