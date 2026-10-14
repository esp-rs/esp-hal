// Reads the `/hil` and `/test-size` commands. Like the tokens of `cargo xtask
// build` and `test`, their words go in any order: a chip, a package, a keyword,
// and everything else is a test or example name.

const CHIPS = require("../chips.json");
const SOCS = CHIPS.map((c) => c.soc);
const QUICK = CHIPS.filter((c) => c.quick).map((c) => c.soc);

// What an ELF basename or example name can look like, plus `::` for the
// `test::filter` form xtask accepts.
const NAME = /^[A-Za-z0-9_:-]+$/;

// xtask takes these as flags too, but here their values are words like any
// other, so the flags add nothing.
const FLAGS = ["--test", "--tests", "-t", "--package", "--packages"];

const HIL_PACKAGES = new Map([
  ["hil-test", "hil-test"],
  ["test", "hil-test"],
  ["tests", "hil-test"],
  ["hil-test-radio", "hil-test-radio"],
  ["radio", "hil-test-radio"],
]);

// What `cargo xtask build --package` gets, empty for the default `examples`.
const SIZE_PACKAGES = new Map([
  ["examples", ""],
  ["example", ""],
  ["qa-test", "qa-test"],
  ["qa", "qa-test"],
]);

const USAGE = `Usage, with the words of a command in any order. \`/hil\` is the previous spelling and still works.
- \`/test quick\` — run a quick HIL matrix (only \`ESP32-S3\` (Xtensa) and \`ESP32-C6\` (RISC-V) tests)
- \`/test full\` — run the full HIL matrix for all supported chips, \`all\` works too
- \`/test <chip> [<chip> ...]\` — run the HIL tests **only** for the listed chips. Example: \`/test c3 s3\`
- Add test names to run only those, in any position. Example: \`/test esp32 rmt c6 i2c\` or \`/test quick radio_basic::test_scan_doesnt_leak\`
- Add \`radio\` to run radio HIL instead of the default \`hil-test\` package, or both package names to run both. Example: \`/test c6 radio\`
- \`/test-size <example> [<example> ...] <chip> [<chip> ...]\` — run binary size analysis for the examples on the chips. Add \`qa\` for a binary of \`qa-test\`. Example: \`/test-size hello_world c3 c6\` or \`/test-size sleep_timer qa c3\`

Chips can be shortened as in xtask: \`c6\`, \`esp32-c6\` and \`ESP32C6\` all mean \`esp32c6\`. Only the first line of the comment is read, the rest is free text.
If you aren't a repository **member/owner**, you must be **trusted for this PR**.
Maintainers can grant access with a \`trusted-author\` label or with:
\`\`\`
/trust @<login>
\`\`\`
and revoke with:
\`\`\`
/revoke @<login>
\`\`\``;

// `esp32-c6`, `ESP32C6` and `c6` name the same chip as `esp32c6`, as in xtask.
function normalizeChip(word) {
  const name = word.toLowerCase().replace(/[-_]/g, "");
  if (!name) return null;
  const soc = name.startsWith("esp32") ? name : `esp32${name}`;
  return SOCS.includes(soc) ? soc : null;
}

// A misspelled chip would otherwise be taken for a test or example name.
function looksLikeChip(word) {
  return /^esp32/i.test(word) || /^[a-z]\d+$/i.test(word);
}

// Backticks would end the code span the word is quoted in.
function quote(word) {
  return `\`${word.replace(/`/g, "")}\``;
}

// The words after the command on the first line of the comment. Commas
// separate words like spaces do, and `--test=rmt` is `--test rmt`.
function commandWords(body, command) {
  const line = String(body || "").split(/\r?\n/, 1)[0].trim();
  const match = line.match(new RegExp(`^${command}(?:\\s+(.*))?$`, "i"));
  if (!match) return null;
  return (match[1] || "").split(/[\s,=]+/).filter(Boolean);
}

function sortWords(words, { keywords, packages }) {
  const sorted = { chips: [], keywords: [], packages: [], names: [], errors: [] };
  const add = (list, value) => list.includes(value) || list.push(value);
  for (const word of words) {
    const lower = word.toLowerCase();
    const chip = normalizeChip(word);
    if (FLAGS.includes(lower)) continue;
    if (chip) add(sorted.chips, chip);
    else if (keywords.includes(lower)) add(sorted.keywords, lower);
    else if (packages.has(lower)) add(sorted.packages, packages.get(lower));
    else if (word.startsWith("-")) sorted.errors.push(`Unknown option ${quote(word)}.`);
    else if (looksLikeChip(word))
      sorted.errors.push(`Unknown chip ${quote(word)}. Known chips are: ${SOCS.join(", ")}.`);
    else if (!NAME.test(word)) sorted.errors.push(`${quote(word)} is not a test or example name.`);
    else add(sorted.names, word);
  }
  return sorted;
}

// `null` for a comment that is not a `/hil` or `/test` command. `help` when it asks
// for the usage, `error` when it can't be run. `/test` stops at a word boundary, so
// `/test-size` is not one of these.
function parseHilCommand(body) {
  const words = ["/hil", "/test"].map((command) => commandWords(body, command)).find(Boolean);
  if (!words) return null;

  const result = { help: false, error: "", chips: "", selector: "", tests: "", package: "" };
  const sorted = sortWords(words, {
    keywords: ["quick", "full", "all", "help"],
    packages: HIL_PACKAGES,
  });
  if (words.length === 0 || sorted.keywords.includes("help")) return { ...result, help: true };
  if (sorted.errors.length) return { ...result, error: sorted.errors.join("\n") };

  const full = sorted.keywords.includes("full") || sorted.keywords.includes("all");
  const quick = sorted.keywords.includes("quick");
  const chips = SOCS.filter(
    (soc) => full || (quick && QUICK.includes(soc)) || sorted.chips.includes(soc),
  );
  if (chips.length === 0) {
    return { ...result, error: "Name the chips to test, like `c6` or `esp32s3`, or `quick` or `full`." };
  }

  let selector = chips.join(" ");
  if (full) selector = "full";
  else if (quick && sorted.chips.every((soc) => QUICK.includes(soc))) selector = "quick";

  return {
    ...result,
    chips: chips.join(" "),
    selector,
    tests: sorted.names.join(","),
    package: sorted.packages.length > 1 ? "all" : sorted.packages[0] || "hil-test",
  };
}

// `null` for a comment that is not a `/test-size` command, `error` when it
// can't be run.
function parseTestSizeCommand(body) {
  const words = commandWords(body, "/test-size");
  if (!words) return null;

  const sorted = sortWords(words, { keywords: [], packages: SIZE_PACKAGES });
  const errors = [...sorted.errors];
  if (sorted.names.length === 0) errors.push("Name the example to measure, like `hello_world`.");
  if (sorted.chips.length === 0) errors.push("Name at least one chip, like `c6` or `esp32s3`.");
  if (sorted.packages.length > 1) errors.push("Examples and `qa-test` binaries are measured one package at a time.");

  return {
    error: errors.join("\n"),
    examples: sorted.names.join(" "),
    package: sorted.packages[0] || "",
    chips: SOCS.filter((soc) => sorted.chips.includes(soc)),
  };
}

module.exports = { parseHilCommand, parseTestSizeCommand, USAGE };
