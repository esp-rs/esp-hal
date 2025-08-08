use crate::cfg::Value;

impl super::RsaProperties {
    pub(super) fn computed_properties(&self) -> impl Iterator<Item = (&str, bool, Value)> {
        let increment = self.size_increment;
        let memory_bits = self.memory_size_bytes * 8;

        [
            (
                "rsa.exponentiation",
                false,
                Value::NumberList(
                    (increment..=memory_bits)
                        .step_by(increment as usize)
                        .collect(),
                ),
            ),
            (
                "rsa.multiplication",
                false,
                Value::NumberList(
                    (increment..=memory_bits / 2)
                        .step_by(increment as usize)
                        .collect(),
                ),
            ),
        ]
        .into_iter()
    }
}
