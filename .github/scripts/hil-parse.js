const DEFAULT_ALLOWED = require("../chips.json").map((c) => c.soc);

// Longest first, because "hil-test" is a prefix of "hil-test-radio".
const PACKAGES = ["hil-test-radio", "hil-test"];

// What an ELF basename can look like, plus `::` for the `test::filter` form
// xtask accepts.
const TEST_NAME = /^[A-Za-z0-9_:-]+$/;

function parsePackage(body) {
  const text = String(body || "").trim().toLowerCase();
  return PACKAGES.find((pkg) => text.includes(pkg)) || "hil-test";
}

function parseTests(body) {
  const text = String(body || "").trim();
  const m = text.match(/--tests?\s+(.+)$/i);
  if (!m) return "";
  return m[1]
    .trim()
    .split(/[,\s]+/)
    .map((s) => s.trim())
    .filter(Boolean)
    // `--tests` runs to the end of the line, so a package selector written
    // after it would otherwise be picked up as a test name.
    .filter((s) => !PACKAGES.includes(s.toLowerCase()))
    .filter((s) => TEST_NAME.test(s))
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

  const chips = Array.from(new Set(parts.filter((s) => allowed.includes(s))));

  if (!chips.length) {
    return {
      chips: "",
      chipsLabel: "",
      error:
        "No valid chips specified.\n\nAllowed chips are: " + allowed.join(", "),
    };
  }

  return {
    chips: chips.join(" "),
    chipsLabel: chips.join(", "),
    error: "",
  };
}

module.exports = { parseTests, parseChips, parsePackage };