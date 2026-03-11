const DEFAULT_ALLOWED = [
  'esp32c2','esp32c3','esp32c5','esp32c6','esp32h2','esp32','esp32s2','esp32s3'
];

function parseTests(body) {
  const text = String(body || "").trim();
  const m = text.match(/--tests?\s+(.+)$/i);
  if (!m) return "";
  return m[1]
    .trim()
    .split(/[,\s]+/)
    .map((s) => s.trim())
    .filter(Boolean)
    .join(",");
}

function parseChips(body, allowed = DEFAULT_ALLOWED) {
  const body_trimmed = String(body || "").trim();

  // Remove the leading "/hil"
  const withoutCmd = body_trimmed.replace(/^\/hil\s+/i, "");

 // Split on commas and/or whitespace, normalize and dedupe
  const parts = withoutCmd
    .split(/[,\s]+/)
    .map((s) => s.toLowerCase().replace(/[,]+$/, ""))
    .filter(Boolean);

  const chips = Array.from(new Set(
    parts.filter(s => allowed.includes(s))
  ));

  if (!chips.length) {
    return {
      chips: "",
      chipsLabel: "",
      error: "No valid chips specified.\n\nAllowed chips are: " + allowed.join(", "),
    };
  }

  return {
    chips: chips.join(" "),
    chipsLabel: chips.join(", "),
    error: "",
  };
}

module.exports = { parseTests, parseChips };
