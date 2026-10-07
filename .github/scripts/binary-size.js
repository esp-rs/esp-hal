// Nightly binary size check for `binary-size-nightly.yml`. `resolveRange`
// picks the commits to compare, `report` compares the measured sizes.

const fs = require("fs");
const path = require("path");

// All reports go to the one open issue with this title.
const ISSUE_TITLE = "Binary size regression on main";
// Sizes downloaded from the `measure` jobs.
const SIZES_DIR = "sizes";

// Runs git and returns its trimmed output.
async function git(exec, args) {
  const { stdout } = await exec.getExecOutput("git", args, { silent: true });
  return stdout.trim();
}

// Base is the given commit, or the last `main` commit older than 24 hours.
// `--first-parent` keeps to `main` itself: one commit per merged pull request.
async function resolveRange({ core, exec }) {
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

// Pairs the "<chip> <example> <base|head> <flash> <bss>" lines into one entry
// per chip and example, sorted so tables keep the same order.
function readSizes() {
  const examples = new Map();
  for (const file of fs.readdirSync(SIZES_DIR)) {
    const text = fs.readFileSync(path.join(SIZES_DIR, file), "utf8");
    for (const line of text.split("\n")) {
      if (!line.trim()) continue;
      const [chip, example, role, flash, bss] = line.trim().split(/\s+/);
      const key = `${chip} ${example}`;
      if (!examples.has(key)) examples.set(key, { chip, example });
      examples.get(key)[role] = { flash: Number(flash), bss: Number(bss) };
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

// Formats a cell like `72734 (+4112, +5.99%)`. It regresses, in bold, only
// when it grows by more than both limits.
function compare(was, now, maxBytes, maxPercent) {
  if (was === undefined) return { text: `${now} (new)`, regressed: false };

  const delta = now - was;
  if (delta === 0) return { text: `${now}`, regressed: false };

  const percent = was > 0 ? (100 * delta) / was : 0;
  const sign = delta > 0 ? "+" : "";
  const text = `${now} (${sign}${delta}, ${sign}${percent.toFixed(2)}%)`;
  const regressed = delta > maxBytes && percent > maxPercent;
  return { text: regressed ? `**${text}**` : text, regressed };
}

// Renders compared rows as a Markdown table.
function table(rows) {
  return [
    "| Chip | Example | Flash (bytes) | bss (bytes) |",
    "|---|---|---|---|",
    ...rows.map(
      (row) =>
        `| \`${row.chip}\` | \`${row.example}\` | ${row.flash.text} | ${row.bss.text} |`,
    ),
  ].join("\n");
}

async function report({ github, context, core, exec }) {
  const base = process.env.BASE;
  const head = process.env.HEAD;
  const limits = limitsFromEnv();

  // An example regresses if its flash or its bss does.
  const rows = readSizes().map((entry) => {
    const flash = compare(
      entry.base?.flash,
      entry.head.flash,
      limits.flash,
      limits.percent,
    );
    const bss = compare(entry.base?.bss, entry.head.bss, limits.bss, limits.percent);
    return { ...entry, flash, bss, regressed: flash.regressed || bss.regressed };
  });
  const regressions = rows.filter((row) => row.regressed);

  // The run summary always gets the full table.
  await core.summary
    .addRaw(
      [
        "## Binary size report",
        "",
        `Base: ${base}, head: ${head}. Regressions are marked in bold.`,
        "",
        table(rows),
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
    `Workflow run: ${context.serverUrl}/${owner}/${repo}/actions/runs/${context.runId}`,
    "",
    "### Regressions",
    "",
    table(regressions),
    "",
    `### Merged between \`${base.slice(0, 10)}\` and \`${head.slice(0, 10)}\``,
    "",
    merged,
    "",
    "<details><summary>All examples</summary>",
    "",
    table(rows),
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

module.exports = { resolveRange, report };
