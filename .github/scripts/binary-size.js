// Binary size checks. `binary-size.yml` measures a pull request on demand
// (`/test-size`), `binary-size-nightly.yml` measures what `main` gained in a
// day or over a chosen date/tag range. Both pick the commits, `measure`
// (through `.github/actions/measure-binary-size`) and report with the
// functions here.

const fs = require("fs");
const path = require("path");

// All nightly reports go to the one open issue with this title.
const ISSUE_TITLE = "Nightly Workflow: Binary size regression on main";
// Marks the pull request comment that holds the report.
const PR_REPORT_HEADING = "## Binary Size Report";
// Sizes written by `measure` (and cached by `prepare`), one file per chip.
const SIZES_DIR = "sizes";
const CACHED_FILE = path.join(SIZES_DIR, "cached.txt");
// Metrics JSON file written by `reportNightly` for `esp-metrics-cli`.
const METRICS_FILE = "binary_size_metrics.json";
const METRICS_NAMES = ["binary_size", "binary_size_sections"];
const MAX_BETWEEN_COMMITS = 15;

// Runs git and returns its trimmed output.
async function git(exec, args) {
  const { stdout } = await exec.getExecOutput("git", args, { silent: true });
  return stdout.trim();
}

async function isAncestor(exec, ancestor, descendant) {
  const { exitCode } = await exec.getExecOutput(
    "git",
    ["merge-base", "--is-ancestor", ancestor, descendant],
    { ignoreReturnCode: true, silent: true },
  );
  return exitCode === 0;
}

function runUrl(context) {
  const { owner, repo } = context.repo;
  return `${context.serverUrl}/${owner}/${repo}/actions/runs/${context.runId}`;
}

function metricsApiConfig(context) {
  const url = (process.env.ESP_METRICS_API_URL || "").trim().replace(/\/+$/, "");
  const token = (process.env.ESP_METRICS_API_TOKEN || "").trim();
  const projectId = Number(
    process.env.GITHUB_REPOSITORY_ID || context.payload.repository?.id || 0,
  );
  const { owner, repo } = context.repo;
  const projectUrl = `${context.serverUrl}/${owner}/${repo}`;
  return url && projectId ? { url, token, projectId, projectUrl } : null;
}

async function postMetricsApi(config, endpoint, body) {
  const headers = { "Content-Type": "application/json" };
  if (config.token) headers.Authorization = `Bearer ${config.token}`;
  const response = await fetch(`${config.url}${endpoint}`, {
    method: "POST",
    headers,
    body: JSON.stringify(body),
  });
  if (response.status === 404) return null;
  if (!response.ok) {
    throw new Error(`${endpoint} returned HTTP ${response.status}`);
  }
  return await response.json();
}

// Queries `/api/v1/metrics/query` for `sha` and returns its stored metrics, or
// null when `sha` has not been uploaded to `app-metrics`.
async function fetchCommitMetrics({ context, core }, sha) {
  const config = metricsApiConfig(context);
  if (!config || !sha) return null;
  try {
    const data = await postMetricsApi(config, "/api/v1/metrics/query", {
      project_id: config.projectId,
      project_url: config.projectUrl,
      commit_sha: sha,
      metrics_names: METRICS_NAMES,
    });
    return data?.success ? data.metrics : null;
  } catch (error) {
    core.warning(`Could not query metrics for ${sha.slice(0, 10)}: ${error.message}`);
    return null;
  }
}

// Queries `/api/v1/metrics/latest_commit` for the latest commit on `main`
// stored in the `longterm` bucket.
async function fetchLatestCommitMetrics({ context, core }) {
  const config = metricsApiConfig(context);
  if (!config) return null;
  try {
    const data = await postMetricsApi(config, "/api/v1/metrics/latest_commit", {
      project_id: config.projectId,
      project_url: config.projectUrl,
      branch_name: "main",
      bucket_type: "longterm",
      metrics_names: METRICS_NAMES,
    });
    return data?.success && data.commit_sha ? data : null;
  } catch (error) {
    core.warning(`Could not query latest commit metrics: ${error.message}`);
    return null;
  }
}

// Converts a `metrics` object from `/api/v1/metrics/query` into the
// `<chip> <example> <role> <section> <flash|bss> <bytes>` lines `groupSizes`
// reads.
function metricsToLines(metrics, role) {
  const sections = metrics?.binary_size_sections?.data || [];
  if (sections.length > 0) {
    return sections.map(
      (s) => `${s.chip} ${s.example} ${role} ${s.section} ${s.kind} ${s.bytes}`,
    );
  }
  const totals = metrics?.binary_size?.data || [];
  return totals.flatMap((t) => [
    `${t.chip} ${t.example} ${role} .flash flash ${t.flash}`,
    `${t.chip} ${t.example} ${role} .bss bss ${t.bss}`,
  ]);
}

// The head of the last successful scheduled run (or the latest commit on `main`
// in `app-metrics`), if `main` still contains it.
async function previousHead({ github, context, core, exec }) {
  const { data } = await github.rest.actions.listWorkflowRuns({
    ...context.repo,
    workflow_id: "binary-size-nightly.yml",
    event: "schedule",
    status: "success",
    per_page: 1,
  });
  const sha = data.workflow_runs[0]?.head_sha;
  if (sha && (await isAncestor(exec, sha, "HEAD"))) return sha;

  const latest = await fetchLatestCommitMetrics({ context, core });
  if (latest?.commit_sha && (await isAncestor(exec, latest.commit_sha, "HEAD"))) {
    return latest.commit_sha;
  }
  return "";
}

// Resolves a date/time window on `main`, preferring a commit near the boundary
// that already has stored metrics in `app-metrics`.
async function resolveBeforeWithCache(api, before, head = "HEAD") {
  const { exec } = api;
  const out = await git(exec, [
    "rev-list",
    "--max-count=10",
    "--first-parent",
    `--before=${before}`,
    head,
  ]);
  const candidates = out.split("\n").filter(Boolean);
  if (candidates.length === 0) return "";
  if (metricsApiConfig(api.context)) {
    for (const sha of candidates) {
      const metrics = await fetchCommitMetrics(api, sha);
      if (metrics && metricsToLines(metrics, "base").length > 0) return sha;
    }
  }
  return candidates[0];
}

// Resolves a date (`YYYY-MM-DD`), release tag (`esp-hal-v1.2.0` / `v1.2.0`),
// or commit hash to a commit SHA.
async function resolveInput(api, input, isStart, head = "HEAD") {
  const { exec } = api;
  if (/^\d{4}-\d{2}-\d{2}$/.test(input)) {
    const time = `${input}T${isStart ? "00:00:00" : "23:59:59"}Z`;
    const sha =
      (await resolveBeforeWithCache(api, time, head)) ||
      (await resolveBeforeWithCache(api, `${input}T23:59:59Z`, head));
    if (!sha) throw new Error(`No commit on main before ${input}.`);
    return sha;
  }
  for (const ref of [input, `esp-hal-${input}`, `esp-hal-v${input}`]) {
    const { exitCode, stdout } = await exec.getExecOutput(
      "git",
      ["rev-parse", "--verify", "--quiet", `${ref}^{commit}`],
      { ignoreReturnCode: true, silent: true },
    );
    if (exitCode === 0) return stdout.trim();
  }
  throw new Error(`Unknown commit, tag, or date: ${input}`);
}

// Picks the `base` and `head` commits to compare, prefetches any sizes already
// stored in `app-metrics`, and outputs the chips that still need to be built.
// `--first-parent` keeps to `main` itself: one commit per merged pull request.
async function resolveNightlyRange({ github, context, core, exec }) {
  const api = { github, context, core, exec };
  const range = process.env.INPUT_RANGE || "since-last-run";
  const baseInput = (process.env.INPUT_BASE || "").trim();
  const headInput = (process.env.INPUT_HEAD || "").trim();

  fs.rmSync(SIZES_DIR, { recursive: true, force: true });

  const head = headInput
    ? await resolveInput(api, headInput, false)
    : await git(exec, ["rev-parse", "HEAD"]);

  let base = "";
  if (baseInput) {
    base = await resolveInput(api, baseInput, true, head);
  } else if (range === "since-last-release") {
    const tag = await git(exec, [
      "describe",
      "--tags",
      "--match",
      "esp-hal-v*",
      "--exclude",
      "esp-hal-v*-*",
      "--abbrev=0",
      head,
    ]);
    base = await resolveInput(api, tag, true, head);
  } else if (range === "last-7-days" || range === "last-30-days") {
    const days = range === "last-7-days" ? 7 : 30;
    base = await resolveBeforeWithCache(api, `${days} days ago`, head);
  } else {
    base =
      (await previousHead(api)) ||
      (await resolveBeforeWithCache(api, "24 hours ago", head));
  }
  core.info(`Comparing ${base} with ${head}`);

  // Empty outputs skip the other jobs.
  if (base === head) {
    core.info(`Nothing was merged since ${base}.`);
    return;
  }

  if (await isAncestor(exec, head, base)) {
    throw new Error(
      `Start commit ${base.slice(0, 10)} is newer than end commit ${head.slice(0, 10)}.`,
    );
  }

  // The commits merged before `head`, oldest first. `measure` builds them only
  // for examples that grew past the limits, to find the pull request behind it.
  const between = (await isAncestor(exec, base, head))
    ? (await git(exec, ["rev-list", "--first-parent", "--reverse", `${base}..${head}`]))
        .split("\n")
        .slice(0, -1)
    : [];
  const shortBetween = between.length <= MAX_BETWEEN_COMMITS ? between : [];

  const cachedLines = [];
  for (const [role, sha] of [
    ["base", base],
    ["head", head],
    ...shortBetween.map((sha) => [sha, sha]),
  ]) {
    const metrics = await fetchCommitMetrics(api, sha);
    const lines = metricsToLines(metrics, role);
    if (lines.length > 0) {
      core.info(`Fetched stored metrics from app-metrics for ${role} (${sha.slice(0, 10)}).`);
      cachedLines.push(...lines);
    }
  }

  if (cachedLines.length > 0) {
    fs.mkdirSync(SIZES_DIR, { recursive: true });
    fs.writeFileSync(CACHED_FILE, `${cachedLines.join("\n")}\n`);
    core.setOutput("has_cached", "true");
  }

  const allChips = JSON.parse(fs.readFileSync(".github/chips.json", "utf8")).map(
    (chip) => chip.soc,
  );
  core.setOutput("base", base);
  core.setOutput("head", head);
  core.setOutput("between", shortBetween.join(" "));
  core.setOutput("all_chips", JSON.stringify(allChips));

  // Determine which chips still need a `measure` job: any chip missing `base`
  // or `head` in `cachedLines`, or any chip that regressed between cached
  // `base` and `head` and has uncached `between` commits to bisect.
  const rolesByChip = new Map(allChips.map((soc) => [soc, new Set()]));
  for (const line of cachedLines) {
    const [chip, , role] = line.trim().split(/\s+/);
    rolesByChip.get(chip)?.add(role);
  }
  const entries = groupSizes(cachedLines).filter((entry) => entry.head);
  const regressedChips = new Set(
    compareEntries(entries, limitsFromEnv())
      .filter((row) => row.regressed)
      .map((row) => row.chip),
  );
  const chipsToBuild = allChips.filter((soc) => {
    const roles = rolesByChip.get(soc);
    if (!roles.has("base") || !roles.has("head")) return true;
    return regressedChips.has(soc) && shortBetween.some((sha) => !roles.has(sha));
  });

  if (chipsToBuild.length === 0) {
    core.info("All required measurements are already in app-metrics; skipping build.");
    return;
  }
  core.setOutput("chips", JSON.stringify(chipsToBuild));
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

// Builds the examples for one chip at the base and at the head commit (skipping
// any commit already fetched from `app-metrics` in `sizes/cached.txt`) and
// writes their newly built sections to `sizes/<soc>.txt`. Node keeps this
// script in memory, so checking out other commits under it is fine.
async function measure({ core, exec }) {
  const { SOC: soc, TARGET: target, BASE: base, HEAD: head } = process.env;
  const pkg = process.env.PACKAGE || "";
  const examples = process.env.EXAMPLES.split(/\s+/).filter(Boolean);
  const between = (process.env.BETWEEN || "").split(/\s+/).filter(Boolean);

  const cachedSoc = fs.existsSync(CACHED_FILE)
    ? fs
        .readFileSync(CACHED_FILE, "utf8")
        .split("\n")
        .filter((line) => line.startsWith(`${soc} `))
    : [];
  const cachedRoles = new Set(cachedSoc.map((line) => line.trim().split(/\s+/)[2]));

  // A pull request head may only be reachable from `refs/pull/*`.
  await exec.exec("git", ["fetch", "--quiet", "--no-tags", "origin", base, head]);

  // Older commits may lack this action, and its post steps need it on disk.
  const original = await git(exec, ["rev-parse", "HEAD"]);
  const built = [];
  try {
    for (const [role, commit] of [
      ["base", base],
      ["head", head],
    ]) {
      if (cachedRoles.has(role)) {
        core.info(
          `Using stored metrics from app-metrics for ${soc} at ${role} (${commit.slice(0, 10)}).`,
        );
        continue;
      }
      built.push(
        ...(await measureCommit({ core, exec, soc, target, pkg, examples, role, commit })),
      );
    }
    const uncachedBetween = between.filter((sha) => !cachedRoles.has(sha));
    built.push(
      ...(await measureBetween({
        core,
        exec,
        soc,
        target,
        pkg,
        lines: [...cachedSoc, ...built],
        between: uncachedBetween,
      })),
    );
  } finally {
    await exec.exec("git", ["checkout", "--quiet", "--force", original]);
  }

  if (built.length === 0) {
    core.info(`Nothing new was built for ${soc}.`);
    return;
  }
  fs.mkdirSync(SIZES_DIR, { recursive: true });
  fs.writeFileSync(path.join(SIZES_DIR, `${soc}.txt`), `${built.join("\n")}\n`);
  core.info(built.join("\n"));
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
    try {
      await exec.exec("cargo", ["xtask", "build", example, ...packageArgs, soc]);
    } catch (error) {
      if (role === "head") throw error;
      core.warning(`Skipping ${example} at ${commit} on ${soc}: ${error.message}`);
      continue;
    }

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

// Builds the examples that grew past the limits at every commit in between,
// so the report can name the pull request behind the growth. A commit that
// fails to build is skipped, its growth then shows up in the next one.
async function measureBetween({ core, exec, soc, target, pkg, lines, between }) {
  if (between.length === 0) return [];
  const entries = groupSizes(lines).filter((entry) => entry.head);
  const examples = compareEntries(entries, limitsFromEnv())
    .filter((row) => row.regressed)
    .map((row) => row.example);
  if (examples.length === 0) return [];

  await exec.exec("git", ["fetch", "--quiet", "--no-tags", "origin", ...between]);
  const found = [];
  for (const commit of between) {
    try {
      found.push(
        ...(await measureCommit({ core, exec, soc, target, pkg, examples, role: commit, commit })),
      );
    } catch (error) {
      core.warning(`Skipping ${commit} on ${soc}: ${error.message}`);
    }
  }
  return found;
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

// Groups the "<chip> <example> <role> <section> <flash|bss> <bytes>" lines into
// one entry per chip and example, with the sections and totals of each commit.
// The role is `base`, `head`, or the hash of a commit in between.
function groupSizes(lines) {
  const examples = new Map();
  for (const line of lines) {
    if (!line.trim()) continue;
    const [chip, example, role, section, kind, bytes] = line.trim().split(/\s+/);
    const key = `${chip} ${example}`;
    if (!examples.has(key)) {
      examples.set(key, { chip, example, kinds: new Map(), between: {} });
    }
    const entry = examples.get(key);
    entry.kinds.set(section, kind);
    const sizes = role === "base" || role === "head" ? entry : entry.between;
    sizes[role] ??= { flash: 0, bss: 0, sections: new Map() };
    if (sizes[role].sections.has(section)) continue;
    sizes[role][kind] += Number(bytes);
    sizes[role].sections.set(section, Number(bytes));
  }
  return [...examples.values()];
}

// Reads the sizes of all chips, sorted so tables keep the same order.
function readSizes() {
  if (!fs.existsSync(SIZES_DIR)) return [];

  const lines = fs
    .readdirSync(SIZES_DIR)
    .flatMap((file) => fs.readFileSync(path.join(SIZES_DIR, file), "utf8").split("\n"));

  // An example that only exists at the base was removed, nothing to report.
  return groupSizes(lines)
    .filter((entry) => entry.head)
    .sort(
      (a, b) =>
        a.chip.localeCompare(b.chip) || a.example.localeCompare(b.example),
    );
}

// Writes `binary_size_metrics.json` in the format `esp-metrics-cli` expects
// when `head` is the current `HEAD` commit and was freshly measured across all
// chips (because `esp-metrics-cli` stamps uploads with the current time).
async function writeMetrics(exec, entries, head, missing) {
  if (entries.length === 0 || missing.length > 0) return false;
  const currentHead = await git(exec, ["rev-parse", "HEAD"]);
  if (head !== currentHead) return false;

  if (fs.existsSync(CACHED_FILE)) {
    const cachedHead = fs
      .readFileSync(CACHED_FILE, "utf8")
      .split("\n")
      .some((line) => line.trim().split(/\s+/)[2] === "head");
    if (cachedHead) return false;
  }

  const totals = [];
  const sections = [];
  for (const entry of entries) {
    totals.push({
      chip: entry.chip,
      example: entry.example,
      flash: entry.head.flash,
      bss: entry.head.bss,
    });
    for (const [section, bytes] of entry.head.sections) {
      sections.push({
        chip: entry.chip,
        example: entry.example,
        section,
        kind: entry.kinds.get(section),
        bytes,
      });
    }
  }
  if (totals.length === 0 || sections.length === 0) return false;

  fs.writeFileSync(
    METRICS_FILE,
    `${JSON.stringify({ binary_size: { data: totals }, binary_size_sections: { data: sections } }, null, 2)}\n`,
  );
  return true;
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

// Formats a cell like `72734 (+4112, +5.99%)`. With limits, regressions and
// reductions that cross both thresholds are marked in bold.
function compare(was, now, maxBytes, maxPercent) {
  if (was === undefined) {
    return { text: `${now} (new)`, delta: 0, regressed: false, improved: false };
  }

  const delta = now - was;
  if (delta === 0) {
    return { text: `${now}`, delta: 0, regressed: false, improved: false };
  }

  const percent = was > 0 ? (100 * delta) / was : 0;
  const sign = delta > 0 ? "+" : "";
  const text = `${now} (${sign}${delta}, ${sign}${percent.toFixed(2)}%)`;
  const regressed =
    maxBytes !== undefined && delta > maxBytes && percent > maxPercent;
  const improved =
    maxBytes !== undefined && -delta > maxBytes && -percent > maxPercent;
  return {
    text: regressed || improved ? `**${text}**` : text,
    delta,
    regressed,
    improved,
  };
}

// Compares the totals of every entry. Without limits nothing regresses.
function compareEntries(entries, limits) {
  return entries.map((entry) => {
    const flash = compare(entry.base?.flash, entry.head.flash, limits?.flash, limits?.percent);
    const bss = compare(entry.base?.bss, entry.head.bss, limits?.bss, limits?.percent);
    const regressed = flash.regressed || bss.regressed;
    const improved = flash.improved || bss.improved;
    const shrank = !regressed && (improved || flash.delta + bss.delta < 0);
    return { ...entry, flash, bss, regressed, improved, shrank };
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

// Names a commit by its pull request, or by its hash and subject when it has
// none. Escaped for a table cell, with mentions kept from notifying anyone.
async function describeCommit({ github, context, exec }, sha) {
  const { data: pulls } = await github.rest.repos.listPullRequestsAssociatedWithCommit({
    ...context.repo,
    commit_sha: sha,
  });
  const pull = pulls.find((pull) => pull.merged_at);
  const text = pull
    ? `#${pull.number} ${pull.title}`
    : `${sha.slice(0, 10)} ${await git(exec, ["log", "-1", "--format=%s", sha])}`;
  return text.replaceAll("@", "@\u200b").replaceAll("|", "\\|");
}

// Splits every regression into the growth of each merged pull request, from
// the sizes `measureBetween` recorded. Commits without sizes are counted
// together with the next one that has them. Returns the table, the examples
// each pull request alone grew past the limits, and the regressions that no
// single pull request explains.
async function growthByPullRequest(api, rows, chain, limits) {
  const names = new Map();
  const lines = [];
  const culprits = new Map();
  const unexplained = [];
  for (const row of rows) {
    if (!row.base) continue;
    let previous = row.base;
    let pending = [];
    const explained = { flash: false, bss: false };
    for (const commit of chain.slice(1)) {
      pending.push(commit);
      const sizes = commit === chain.at(-1) ? row.head : row.between[commit];
      if (!sizes) continue;

      if (sizes.flash !== previous.flash || sizes.bss !== previous.bss) {
        for (const sha of pending) {
          if (!names.has(sha)) names.set(sha, await describeCommit(api, sha));
        }
        const flash = compare(previous.flash, sizes.flash, limits.flash, limits.percent);
        const bss = compare(previous.bss, sizes.bss, limits.bss, limits.percent);
        const pulls = pending.map((sha) => names.get(sha)).join("<br>");
        lines.push(
          `| \`${row.chip}\` | \`${row.example}\` | ${pulls} | ${flash.text} | ${bss.text} |`,
        );

        if (flash.regressed || bss.regressed) {
          const culprit = pending.map((sha) => names.get(sha)).join(" or ");
          if (!culprits.has(culprit)) culprits.set(culprit, []);
          culprits.get(culprit).push(row);
        }
        explained.flash ||= flash.regressed;
        explained.bss ||= bss.regressed;
      }
      previous = sizes;
      pending = [];
    }
    if ((row.flash.regressed && !explained.flash) || (row.bss.regressed && !explained.bss)) {
      unexplained.push(row);
    }
  }
  const table = [
    "| Chip | Example | Pull request | Flash (bytes) | bss (bytes) |",
    "|---|---|---|---|---|",
    ...lines,
  ].join("\n");
  return { table, hasRows: lines.length > 0, culprits, unexplained };
}

// Lists rows as "`example` on `chip`, `chip`; `example` on `chip`".
function whereText(rows) {
  const chips = new Map();
  for (const row of rows) {
    if (!chips.has(row.example)) chips.set(row.example, []);
    chips.get(row.example).push(`\`${row.chip}\``);
  }
  return [...chips]
    .map(([example, list]) => `\`${example}\` on ${list.join(", ")}`)
    .join("; ");
}

// Names the pull requests that alone grew past the limits, and where, and the
// regressions that only add up over several pull requests.
function causeText({ culprits, unexplained }) {
  const lines = [...culprits].map(
    ([culprit, rows]) => `**${culprit}** grew past the limits on its own: ${whereText(rows)}.`,
  );
  if (unexplained.length > 0) {
    lines.push(
      `No single pull request grew past the limits on its own in ${whereText(unexplained)}, the growth adds up over several of them. See the growth by pull request below.`,
    );
  }
  return lines.length === 1 ? lines[0] : lines.map((line) => `- ${line}`).join("\n");
}

// Writes the nightly table to the run summary, exports `binary_size_metrics.json`
// for `esp-metrics-cli`, and on scheduled runs opens or updates the regression
// issue when an example grew past the limits.
async function reportNightly({ github, context, core, exec }) {
  const base = process.env.BASE;
  const head = process.env.HEAD;
  const limits = limitsFromEnv();

  const entries = readSizes();
  const missing = missingChips(entries);
  const hasMetrics = await writeMetrics(exec, entries, head, missing);
  core.setOutput("has_metrics", hasMetrics ? "true" : "false");

  const rows = compareEntries(entries, limits);
  const regressions = rows.filter((row) => row.regressed);
  const improvements = rows.filter((row) => row.shrank);
  const missingText = missingNote(missing);

  const between = (process.env.BETWEEN || "").split(/\s+/).filter(Boolean);
  const growth =
    regressions.length > 0
      ? await growthByPullRequest(
          { github, context, exec },
          regressions,
          [base, ...between, head],
          limits,
        )
      : null;

  const summary = [
    "## Binary size report",
    "",
    `Base: \`${base.slice(0, 10)}\`, head: \`${head.slice(0, 10)}\`. Changes past the limits are marked in bold.`,
    "",
    ...(missingText ? [missingText, ""] : []),
  ];

  if (regressions.length > 0) {
    summary.push("### Cause", "", causeText(growth), "");
    summary.push("### Regressions", "", totalsTable(regressions), "", sectionsDetails(regressions), "");
    if (growth.hasRows) {
      summary.push(
        "### Growth by pull request",
        "",
        "The size after each pull request that changed it. In bold where that pull request alone grew past the limits.",
        "",
        growth.table,
        "",
      );
    }
  }

  if (improvements.length > 0) {
    summary.push("### Improvements", "", totalsTable(improvements), "", sectionsDetails(improvements), "");
  }

  summary.push(
    "<details><summary>All examples</summary>",
    "",
    totalsTable(rows),
    "",
    sectionsDetails(rows),
    "",
    "</details>",
  );

  await core.summary.addRaw(summary.join("\n")).write();

  if (regressions.length === 0) {
    core.info("No example grew past the limits.");
    return;
  }

  if (context.eventName !== "schedule") {
    core.info("Manual run: skipping issue creation.");
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
    "### Cause",
    "",
    causeText(growth),
    "",
    "### Regressions",
    "",
    totalsTable(regressions),
    "",
    sectionsDetails(regressions),
    "",
    "### Growth by pull request",
    "",
    "The size after each pull request that changed it. In bold where that pull request alone grew past the limits.",
    "",
    growth.table,
    "",
    ...(improvements.length > 0
      ? ["### Improvements", "", totalsTable(improvements), "", sectionsDetails(improvements), ""]
      : []),
    `### Merged between \`${base.slice(0, 10)}\` and \`${head.slice(0, 10)}\``,
    "",
    merged,
    "",
    ...(missingText ? [missingText, ""] : []),
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
