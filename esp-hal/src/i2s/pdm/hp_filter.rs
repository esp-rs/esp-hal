//! High-pass filter coefficient lookup for PDM PCM filters.

/// Returns `(param0, param5)` for the closest cut-off frequency.
///
/// `freq_hz_x10` is the cut-off frequency multiplied by 10 (e.g. 355 for 35.5 Hz).
pub(crate) fn cut_off_coefficients(freq_hz_x10: u32) -> (u32, u32) {
    const TABLE: [[u32; 3]; 21] = [
        [1850, 0, 0],
        [1720, 0, 1],
        [1600, 1, 1],
        [1500, 1, 2],
        [1370, 2, 2],
        [1260, 2, 3],
        [1200, 0, 3],
        [1150, 3, 3],
        [1060, 1, 7],
        [1040, 2, 4],
        [920, 4, 4],
        [915, 2, 7],
        [810, 4, 5],
        [772, 3, 7],
        [690, 5, 5],
        [630, 4, 7],
        [580, 5, 6],
        [490, 5, 7],
        [460, 6, 6],
        [355, 6, 7],
        [233, 7, 7],
    ];

    let mut best = 0;
    let mut min_diff = u32::MAX;
    for (i, row) in TABLE.iter().enumerate() {
        let diff = row[0].abs_diff(freq_hz_x10);
        if diff < min_diff {
            min_diff = diff;
            best = i;
        }
    }

    (TABLE[best][1], TABLE[best][2])
}
