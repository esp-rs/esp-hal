use std::fmt::Write;

use crate::Value;

pub(crate) const DOC_TABLE_HEADER: &str = r#"
| Name | Description | Default value | Allowed value |
|------|-------------|---------------|---------------|
"#;

pub(crate) const SELECTED_TABLE_HEADER: &str = r#"
| Name | Selected value |
|------|----------------|
"#;

pub(crate) fn write_doc_table_line(
    mut table: impl Write,
    name: &str,
    description: &str,
    default: &Value,
    allowed_values: &str,
) {
    writeln!(
        table,
        "|**{name}**|{description}|{default}|{allowed_values}"
    )
    .unwrap();
}

pub(crate) fn write_summary_table_line(mut table: impl Write, name: &str, value: &Value) {
    writeln!(table, "|**{name}**|{value}|").unwrap();
}
