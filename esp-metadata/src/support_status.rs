use std::{fmt, marker::PhantomData, str::FromStr};

use serde::{
    Deserialize,
    Deserializer,
    Serialize,
    de::{self, MapAccess, Visitor},
};

#[derive(Debug, Default, Clone, Copy, PartialEq, Deserialize, Serialize)]
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

impl FromStr for SupportStatusLevel {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "not_available" => Ok(Self::NotAvailable),
            "not_supported" => Ok(Self::NotSupported),
            "partial" => Ok(Self::Partial),
            "supported" => Ok(Self::Supported),
            _ => Err(()),
        }
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub(crate) struct SupportStatus {
    #[serde(default)]
    pub status: SupportStatusLevel,

    #[serde(default)]
    pub issue: Option<u32>,
}

impl FromStr for SupportStatus {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Ok(Self {
            status: s.parse()?,
            issue: None,
        })
    }
}

// https://serde.rs/string-or-struct.html
pub(crate) fn string_or_struct<'de, T, D>(deserializer: D) -> Result<T, D::Error>
where
    T: Deserialize<'de> + FromStr<Err = ()>,
    D: Deserializer<'de>,
{
    // This is a Visitor that forwards string types to T's `FromStr` impl and
    // forwards map types to T's `Deserialize` impl. The `PhantomData` is to
    // keep the compiler from complaining about T being an unused generic type
    // parameter. We need T in order to know the Value type for the Visitor
    // impl.
    struct StringOrStruct<T>(PhantomData<fn() -> T>);

    impl<'de, T> Visitor<'de> for StringOrStruct<T>
    where
        T: Deserialize<'de> + FromStr<Err = ()>,
    {
        type Value = T;

        fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
            formatter.write_str("string or map")
        }

        fn visit_str<E>(self, value: &str) -> Result<T, E>
        where
            E: de::Error,
        {
            FromStr::from_str(value).map_err(|_| E::custom("invalid string"))
        }

        fn visit_map<M>(self, map: M) -> Result<T, M::Error>
        where
            M: MapAccess<'de>,
        {
            // `MapAccessDeserializer` is a wrapper that turns a `MapAccess`
            // into a `Deserializer`, allowing it to be used as the input to T's
            // `Deserialize` implementation. T then deserializes itself using
            // the entries from the map visitor.
            Deserialize::deserialize(de::value::MapAccessDeserializer::new(map))
        }
    }

    deserializer.deserialize_any(StringOrStruct(PhantomData))
}
