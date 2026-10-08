// Binary size checks. `binary-size.yml` measures a pull request on demand
// (`/test-size`), `binary-size-nightly.yml` measures what `main` gained in a
// day. Both pick the commits, `measure` (through
// `.github/actions/measure-binary-size`) and report with the functions here.

const fs = require("fs");
const path = require("path");

// All nightly reports go to the one open issue with this title.
const ISSUE_TITLE = "Nightly Workflow: Binary size regression on main";
// Marks the pull request comment that holds the report.
const PR_REPORT_HEADING = "## Binary Size Report";
// Sizes written by `measure`, one file per chip.
const SIZES_DIR = "sizes";

// Runs git and returns its trimmed output.
async function git(exec, args) {
  const { stdout } = await exec.getExecOutput("git", args, { silent: true });
  return stdout.trim();
}

function runUrl(context) {
  const { owner, repo } = context.repo;
  return `${context.serverUrl}/${owner}/${repo}/actions/runs/${context.runId}`;
}

// Base is the given commit, or the last `main` commit older than 24 hours.
// `--first-parent` keeps to `main` itself: one commit per merged pull request.
async function resolveNightlyRange({ core, exec }) {
  const input = process.env.INPUT_BASE || "";
  const head = await git(exec, ["rev-parse", "HEAD"]);
  const base = input
    ? await git(exec, ["rev-parse", "--verify", `${input}^{commit}`])
    : await git(exec, [
        "rev-list",
        "-1",
        "--first-parent",
        "--before=24 hours ago",
        "HEAD",
      ]);
  core.info(`Comparing ${base} with ${head}`);

  // Empty outputs skip the other jobs.
  if (base === head) {
    core.info(`Nothing was merged since ${base}.`);
    return;
  }

  // The matrix of the `measure` jobs.
  const chips = JSON.parse(fs.readFileSync(".github/chips.json", "utf8"));
  core.setOutput("base", base);
  core.setOutput("head", head);
  core.setOutput("chips", JSON.stringify(chips.map((chip) => chip.soc)));
}

// Compares the pull request head with the commit it branched from, so changes
// merged into the base branch since then do not show up as its own.
async function resolvePullRequestRange({ github, context, core }) {
  const { owner, repo } = context.repo;
  const { data: pull } = await github.rest.pulls.get({
    owner,
    repo,
    pull_number: Number(process.env.PR_NUMBER),
  });
  const { data: comparison } = await github.rest.repos.compareCommitsWithBasehead({
    owner,
    repo,
    basehead: `${pull.base.ref}...${pull.head.sha}`,
  });
  const base = comparison.merge_base_commit.sha;
  core.info(`Comparing ${base} with ${pull.head.sha}`);
  core.setOutput("base", base);
  core.setOutput("head", pull.head.sha);
}

// Finds `<example>/Cargo.toml` under `dir`, skipping build output.
function findManifest(dir, example) {
  for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
    if (!entry.isDirectory() || entry.name === "target" || entry.name.startsWith(".")) {
      continue;
    }
    const child = path.join(dir, entry.name);
    const manifest = path.join(child, "Cargo.toml");
    if (entry.name === example && fs.existsSync(manifest)) return manifest;
    const found = findManifest(child, example);
    if (found) return found;
  }
  return null;
}

// Returns the possible ELF names of an example, or null when it does not
// support the chip. A standalone example lists its chips as features and names
// its ELF in `[package]`. A binary of a package like `qa` is named after its
// source file, and the build itself rejects an unsupported chip.
function elfNames(example, soc, pkg) {
  const guesses = [example, example.replaceAll("_", "-")];
  const manifest = pkg ? null : findManifest("examples", example);
  if (!manifest) return guesses;

  const text = fs.readFileSync(manifest, "utf8");
  if (!new RegExp(`^${soc}\\s*=`, "m").test(text)) return null;
  const packageSection = text.split(/^\[/m).find((part) => part.startsWith("package]"));
  const name = packageSection?.match(/^name\s*=\s*"([^"]+)"/m)?.[1];
  return name ? [name] : guesses;
}

// Indices of the sections that hold at least one variable or function.
async function sectionsWithSymbols(exec, elf) {
  const { stdout } = await exec.getExecOutput("readelf", ["-s", "-W", elf], {
    silent: true,
  });
  const indices = new Set();
  for (const line of stdout.split("\n")) {
    // "Num: Value Size Type Bind Vis Ndx Name"
    const [num, , , type, , , ndx] = line.trim().split(/\s+/);
    if (/^\d+:$/.test(num) && (type === "OBJECT" || type === "FUNC") && /^\d+$/.test(ndx)) {
      indices.add(Number(ndx));
    }
  }
  if (indices.size === 0) throw new Error(`${elf} has no symbol table.`);
  return indices;
}

// Lists the sections of an ELF file that end up in the image (flag `A`):
// sections with contents count as flash, NOBITS ones as bss. A NOBITS section
// without a single variable in it, like `.stack` or `.text_gap`, only reserves
// address space for the memory layout and is skipped.
async function sectionsOf(exec, elf) {
  const { stdout } = await exec.getExecOutput("readelf", ["-S", "-W", elf], {
    silent: true,
  });
  const withSymbols = await sectionsWithSymbols(exec, elf);
  const sections = [];
  for (const line of stdout.split("\n")) {
    // "[Nr] Name Type Address Off Size ES Flg Lk Inf Al"
    const row = line.match(/^\s*\[\s*(\d+)\]\s+(.*)$/);
    if (!row) continue;
    const [name, type, , , size, , flags] = row[2].trim().split(/\s+/);
    const bytes = parseInt(size, 16);
    if (!flags?.includes("A") || !bytes) continue;
    if (type === "NOBITS" && !withSymbols.has(Number(row[1]))) continue;
    sections.push({ name, kind: type === "NOBITS" ? "bss" : "flash", bytes });
  }
  return sections;
}

// Builds the examples for one chip at the base and at the head commit and
// writes their sections to `sizes/<soc>.txt`. Node keeps this script in
// memory, so checking out other commits under it is fine.
async function measure({ core, exec }) {
  const { SOC: soc, TARGET: target, BASE: base, HEAD: head } = process.env;
  const pkg = process.env.PACKAGE || "";
  const examples = process.env.EXAMPLES.split(/\s+/).filter(Boolean);

  // A pull request head may only be reachable from `refs/pull/*`.
  await exec.exec("git", ["fetch", "--quiet", "--no-tags", "origin", base, head]);

  // Older commits may lack this action, and its post steps need it on disk.
  const original = await git(exec, ["rev-parse", "HEAD"]);
  const lines = [];
  try {
    for (const [role, commit] of [
      ["base", base],
      ["head", head],
    ]) {
      lines.push(...(await measureCommit({ core, exec, soc, target, pkg, examples, role, commit })));
    }
  } finally {
    await exec.exec("git", ["checkout", "--quiet", "--force", original]);
  }

  if (lines.length === 0) {
    core.info(`Nothing was built for ${soc}.`);
    return;
  }
  fs.mkdirSync(SIZES_DIR, { recursive: true });
  fs.writeFileSync(path.join(SIZES_DIR, `${soc}.txt`), `${lines.join("\n")}\n`);
  core.info(lines.join("\n"));
}

// Checks out one commit and returns the section lines of its examples.
async function measureCommit({ core, exec, soc, target, pkg, examples, role, commit }) {
  await exec.exec("git", ["checkout", "--quiet", "--force", commit]);

  const lines = [];
  for (const example of examples) {
    const names = elfNames(example, soc, pkg);
    if (!names) {
      core.info(`Skipping ${example}: it does not support ${soc}.`);
      continue;
    }

    const packageArgs = pkg ? ["--package", pkg] : [];
    await exec.exec("cargo", ["xtask", "build", example, ...packageArgs, soc]);

    const elf = names
      .map((name) => path.join("target", target, "release", name))
      .find((file) => fs.existsSync(file));
    if (!elf) throw new Error(`No ELF found for ${example} on ${soc}.`);

    for (const section of await sectionsOf(exec, elf)) {
      lines.push(`${soc} ${example} ${role} ${section.name} ${section.kind} ${section.bytes}`);
    }
  }
  return lines;
}

// Reads the `MAX_*` limits from `env`. A typo fails the run instead of
// silently never reporting.
function limitsFromEnv() {
  const limits = {
    percent: Number(process.env.MAX_GROWTH_PERCENT),
    flash: Number(process.env.MAX_FLASH_GROWTH_BYTES),
    bss: Number(process.env.MAX_BSS_GROWTH_BYTES),
  };
  for (const [name, value] of Object.entries(limits)) {
    if (!Number.isFinite(value)) {
      throw new Error(`The ${name} growth limit is not a number.`);
    }
  }
  return limits;
}

// Groups the "<chip> <example> <base|head> <section> <flash|bss> <bytes>" lines
// into one entry per chip and example, with the sections and totals of both
// commits. Sorted so tables keep the same order.
function readSizes() {
  if (!fs.existsSync(SIZES_DIR)) return [];

  const examples = new Map();
  for (const file of fs.readdirSync(SIZES_DIR)) {
    const text = fs.readFileSync(path.join(SIZES_DIR, file), "utf8");
    for (const line of text.split("\n")) {
      if (!line.trim()) continue;
      const [chip, example, role, section, kind, bytes] = line.trim().split(/\s+/);
      const key = `${chip} ${example}`;
      if (!examples.has(key)) examples.set(key, { chip, example });
      const entry = examples.get(key);
      entry[role] ??= { flash: 0, bss: 0, sections: new Map() };
      entry[role][kind] += Number(bytes);
      entry[role].sections.set(section, Number(bytes));
    }
  }

  // An example that only exists at the base was removed, nothing to report.
  return [...examples.values()]
    .filter((entry) => entry.head)
    .sort(
      (a, b) =>
        a.chip.localeCompare(b.chip) || a.example.localeCompare(b.example),
    );
}

// Chips that were asked for but produced no sizes: their build failed, or
// none of the examples supports them.
function missingChips(entries) {
  const measured = new Set(entries.map((entry) => entry.chip));
  return JSON.parse(process.env.CHIPS || "[]").filter((chip) => !measured.has(chip));
}

function signed(number) {
  return number > 0 ? `+${number}` : `${number}`;
}

// Formats a cell like `72734 (+4112, +5.99%)`. With limits, it regresses, in
// bold, only when it grows by more than both of them.
function compare(was, now, maxBytes, maxPercent) {
  if (was === undefined) return { text: `${now} (new)`, regressed: false };

  const delta = now - was;
  if (delta === 0) return { text: `${now}`, regressed: false };

  const percent = was > 0 ? (100 * delta) / was : 0;
  const sign = delta > 0 ? "+" : "";
  const text = `${now} (${sign}${delta}, ${sign}${percent.toFixed(2)}%)`;
  const regressed =
    maxBytes !== undefined && delta > maxBytes && percent > maxPercent;
  return { text: regressed ? `**${text}**` : text, regressed };
}

// Compares the totals of every entry. Without limits nothing regresses.
function compareEntries(entries, limits) {
  return entries.map((entry) => {
    const flash = compare(entry.base?.flash, entry.head.flash, limits?.flash, limits?.percent);
    const bss = compare(entry.base?.bss, entry.head.bss, limits?.bss, limits?.percent);
    return { ...entry, flash, bss, regressed: flash.regressed || bss.regressed };
  });
}

// Renders compared entries as a Markdown table of their totals.
function totalsTable(rows) {
  return [
    "| Chip | Example | Flash (bytes) | bss (bytes) |",
    "|---|---|---|---|",
    ...rows.map(
      (row) =>
        `| \`${row.chip}\` | \`${row.example}\` | ${row.flash.text} | ${row.bss.text} |`,
    ),
  ].join("\n");
}

// Lists the sections that changed size, biggest change first, one collapsed
// table per example. Shows where the growth went, e.g. into `.trap`.
function sectionsDetails(rows) {
  const blocks = [];
  for (const row of rows) {
    const before = row.base?.sections ?? new Map();
    const after = row.head.sections;
    const names = new Set([...before.keys(), ...after.keys()]);
    const changed = [...names]
      .map((name) => ({
        name,
        was: before.get(name) ?? 0,
        now: after.get(name) ?? 0,
      }))
      .filter((section) => section.was !== section.now)
      .sort((a, b) => Math.abs(b.now - b.was) - Math.abs(a.now - a.was));
    if (changed.length === 0) continue;

    blocks.push(
      [
        `<details><summary><code>${row.chip}</code> <code>${row.example}</code>: sections that changed</summary>`,
        "",
        "| Section | Before | After | Change |",
        "|---|---|---|---|",
        ...changed.map(
          (section) =>
            `| \`${section.name}\` | ${section.was} | ${section.now} | ${signed(section.now - section.was)} |`,
        ),
        "",
        "</details>",
      ].join("\n"),
    );
  }
  return blocks.join("\n\n");
}

function missingNote(chips) {
  if (chips.length === 0) return "";
  return `No sizes from ${chips.map((chip) => `\`${chip}\``).join(", ")}: the build failed or no example supports them, see the run.`;
}

// Writes the nightly table to the run summary and, when something grew past
// the limits, opens or updates the regression issue.
async function reportNightly({ github, context, core, exec }) {
  const base = process.env.BASE;
  const head = process.env.HEAD;
  const limits = limitsFromEnv();

  const entries = readSizes();
  const rows = compareEntries(entries, limits);
  const regressions = rows.filter((row) => row.regressed);
  const missing = missingNote(missingChips(entries));

  // The run summary always gets the full report.
  await core.summary
    .addRaw(
      [
        "## Binary size report",
        "",
        `Base: ${base}, head: ${head}. Regressions are marked in bold.`,
        "",
        ...(missing ? [missing, ""] : []),
        totalsTable(rows),
        "",
        sectionsDetails(rows),
      ].join("\n"),
    )
    .write();

  if (regressions.length === 0) {
    core.info("No example grew past the limits.");
    return;
  }

  const { owner, repo } = context.repo;
  // One line per merged pull request. A zero-width space after `@` keeps
  // mentions in commit subjects from notifying anyone.
  const merged = (
    await git(exec, ["log", "--first-parent", "--format=- %h %s", `${base}..${head}`])
  ).replaceAll("@", "@\u200b");

  const body = [
    "The nightly binary size check found examples that grew past the limits.",
    "",
    `Workflow run: ${runUrl(context)}`,
    "",
    "### Regressions",
    "",
    totalsTable(regressions),
    "",
    sectionsDetails(regressions),
    "",
    `### Merged between \`${base.slice(0, 10)}\` and \`${head.slice(0, 10)}\``,
    "",
    merged,
    "",
    ...(missing ? [missing, ""] : []),
    "<details><summary>All examples</summary>",
    "",
    totalsTable(rows),
    "",
    "</details>",
  ].join("\n");

  // Comment on the open issue, or open one. The API also lists pull requests.
  const issues = await github.paginate(github.rest.issues.listForRepo, {
    owner,
    repo,
    state: "open",
    creator: "github-actions[bot]",
    per_page: 100,
  });
  const existing = issues.find(
    (issue) => !issue.pull_request && issue.title === ISSUE_TITLE,
  );

  if (existing) {
    await github.rest.issues.createComment({
      owner,
      repo,
      issue_number: existing.number,
      body,
    });
    core.info(`Commented on ${existing.html_url}`);
  } else {
    const { data: issue } = await github.rest.issues.create({
      owner,
      repo,
      title: ISSUE_TITLE,
      body,
    });
    core.info(`Opened ${issue.html_url}`);
  }
}

// Puts the `/test-size` report into the comment that triggered it, or into the
// earlier report comment, or a new one.
async function reportPullRequest({ github, context, core }) {
  const { owner, repo } = context.repo;
  const number = Number(process.env.PR_NUMBER);
  const base = process.env.BASE;
  const head = process.env.HEAD;

  const entries = readSizes();
  const rows = compareEntries(entries);
  const missing = missingNote(missingChips(entries));

  const body = [`Run: ${runUrl(context)}`, "", PR_REPORT_HEADING, ""];
  if (rows.length === 0) {
    body.push("Binary size analysis **failed**, nothing was measured. See the run for details.");
  } else {
    body.push(
      `Head \`${head.slice(0, 10)}\` against \`${base.slice(0, 10)}\`, the commit this pull request branched from.`,
      "",
      ...(missing ? [missing, ""] : []),
      totalsTable(rows),
      "",
      sectionsDetails(rows),
    );
  }

  let commentId = Number(process.env.COMMENT_ID) || 0;
  if (!commentId) {
    const comments = await github.paginate(github.rest.issues.listComments, {
      owner,
      repo,
      issue_number: number,
      per_page: 100,
    });
    const earlier = comments.find(
      (comment) =>
        comment.user?.login === "github-actions[bot]" &&
        String(comment.body || "").includes(PR_REPORT_HEADING),
    );
    commentId = earlier?.id ?? 0;
  }

  if (commentId) {
    await github.rest.issues.updateComment({
      owner,
      repo,
      comment_id: commentId,
      body: body.join("\n"),
    });
    core.info(`Updated comment ${commentId} on #${number}`);
  } else {
    await github.rest.issues.createComment({
      owner,
      repo,
      issue_number: number,
      body: body.join("\n"),
    });
    core.info(`Commented on #${number}`);
  }
}

module.exports = {
  resolveNightlyRange,
  resolvePullRequestRange,
  measure,
  reportNightly,
  reportPullRequest,
};
