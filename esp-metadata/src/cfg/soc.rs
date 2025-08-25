use crate::cfg::Value;

impl super::SocProperties {
    pub(super) fn computed_properties(&self) -> impl Iterator<Item = (&str, bool, Value)> {
        let mut properties = vec![];

        if self.xtal_options.len() > 1 {
            // In this case, the HAL can use `for_each_soc_xtal_options` to see all available
            // options.
            properties.push(("soc.has_multiple_xtal_options", false, Value::Boolean(true)));
        } else {
            properties.push((
                "soc.xtal_frequency",
                false,
                Value::Number(self.xtal_options[0]),
            ));
        }

        properties.into_iter()
    }
}
