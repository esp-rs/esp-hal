use crate::cfg::{GenericProperty, Value};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct RsaLengths {
    increment: u32,
    max: u32,
}

impl RsaLengths {
    fn generate(&self) -> Vec<u32> {
        (self.increment..=self.max)
            .step_by(self.increment as usize)
            .collect()
    }
}

impl GenericProperty for RsaLengths {}

impl super::RsaProperties {
    pub(super) fn computed_properties(&self) -> impl Iterator<Item = (&str, Value)> {
        [
            (
                "rsa.exponentiation",
                Value::NumberList(self.exponentiation.generate()),
            ),
            (
                "rsa.multiplication",
                Value::NumberList(self.multiplication.generate()),
            ),
        ]
        .into_iter()
    }
}
