use std::fmt::Write;

use crate::{ConfigOption, Value};

pub(crate) const DOC_TABLE_HEADER: &str = r#"
| Option | Stability | Default&nbsp;value | Allowed&nbsp;values |
|--------|:---------:|:------------------:|:-------------------:|
"#;

pub(crate) const SELECTED_TABLE_HEADER: &str = r#"
| Name | Selected value |
|------|----------------|
"#;

pub(crate) fn write_doc_table_line(mut table: impl Write, name: &str, option: &ConfigOption) {
    let allowed_values = option
        .constraint
        .as_ref()
        .and_then(|validator| validator.description())
        .unwrap_or_default();

    writeln!(
        table,
        "| <p>**{key}**</p> <p>{description}</p> | {stability} | {default} | {allowed}",
        description = option.description,
        key = name,
        stability = option.stability,
        default = option.display_hint.format_value(&option.default_value),
        allowed = allowed_values
    )
    .unwrap();
}

pub(crate) fn write_summary_table_line(mut table: impl Write, name: &str, value: &Value) {
    writeln!(table, "|**{name}**|{value}|").unwrap();
}
