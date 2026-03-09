use std::fmt;

#[derive(Debug, Default, Clone, Copy, PartialEq, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum SupportStatusLevel {
    NotAvailable,
    NotSupported,
    #[default] // Just the common option to reduce visual noise of "declare only" drivers.
    Partial,
    Supported,
}

impl SupportStatusLevel {
    pub fn icon(self) -> &'static str {
        match self {
            SupportStatusLevel::NotAvailable => "",
            SupportStatusLevel::NotSupported => "❌",
            SupportStatusLevel::Partial => "⚒️",
            SupportStatusLevel::Supported => "✔️",
        }
    }

    pub fn status(self) -> &'static str {
        match self {
            SupportStatusLevel::NotAvailable => "Not available",
            SupportStatusLevel::NotSupported => "Not supported",
            SupportStatusLevel::Partial => "Partial support",
            SupportStatusLevel::Supported => "Supported",
        }
    }

    pub(crate) fn write_legend(output: &mut impl fmt::Write) -> fmt::Result {
        for s in [
            SupportStatusLevel::NotAvailable,
            SupportStatusLevel::NotSupported,
            SupportStatusLevel::Partial,
            SupportStatusLevel::Supported,
        ] {
            writeln!(
                output,
                " * {}: {}",
                if s.icon().is_empty() {
                    "Empty cell"
                } else {
                    s.icon()
                },
                s.status()
            )?;
        }

        Ok(())
    }
}
