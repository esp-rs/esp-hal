use std::fmt::Write;

use crate::{ConfigOption, Value};

pub(crate) const DOC_TABLE_HEADER: &str = r#"
| Name | Description | Default value | Allowed value |
|------|-------------|---------------|---------------|
"#;

pub(crate) const SELECTED_TABLE_HEADER: &str = r#"
| Name | Selected value |
|------|----------------|
"#;

pub(crate) fn write_doc_table_line(mut table: impl Write, prefix: &str, option: &ConfigOption) {
    let name = option.env_var(prefix);

    let allowed_values = option
        .constraint
        .as_ref()
        .and_then(|validator| validator.description())
        .unwrap_or(String::from("-"));

    writeln!(
        table,
        "|**{}**|{}|{}|{}",
        name, option.description, option.default_value, allowed_values
    )
    .unwrap();
}

pub(crate) fn write_summary_table_line(mut table: impl Write, name: &str, value: &Value) {
    writeln!(table, "|**{name}**|{value}|").unwrap();
}
