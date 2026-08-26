// Freeze state format, shared by the gate and the freeze/thaw workflow.
//
// State lives in an open issue labelled `merge-freeze`; its `yaml` block holds
// the settings, so a freeze is re-tuned by editing the issue.

const FREEZE_LABEL = "merge-freeze";
const EXEMPT_LABEL = "merge-freeze-exempt";
const AUDIT_MARKER_PREFIX = "<!-- merge-freeze-audit:";

const DEFAULT_SCOPE = ["main"];
const DEFAULT_MIN_APPROVALS = 2;
const DEFAULT_REASON = "pre-release merge freeze";

function normalizeBranch(name) {
  return String(name || "")
    .trim()
    .replace(/^refs\/heads\//, "");
}

function unquote(value) {
  const text = String(value || "").trim();
  const quoted = text.match(/^"(.*)"$/) || text.match(/^'(.*)'$/);
  return quoted ? quoted[1] : text;
}

function parseList(value) {
  const text = String(value || "").trim();
  const inner = text.startsWith("[") && text.endsWith("]")
    ? text.slice(1, -1)
    : text;
  return inner
    .split(",")
    .map((entry) => normalizeBranch(unquote(entry)))
    .filter((entry) => entry.length > 0);
}

// Minimal `key: value` reader instead of a YAML dependency: `github-script` has
// no node_modules, and the config is four scalars plus one inline list.
function parseFreezeConfig(body) {
  const text = String(body || "");
  const block = text.match(/```ya?ml\s*([\s\S]*?)```/i);
  const config = {
    version: "",
    reason: DEFAULT_REASON,
    scope: [...DEFAULT_SCOPE],
    minApprovals: DEFAULT_MIN_APPROVALS,
    effectiveAt: "",
  };

  if (!block) {
    return config;
  }

  for (const rawLine of block[1].split("\n")) {
    const line = rawLine.trim();
    if (!line || line.startsWith("#")) continue;

    const pair = line.match(/^([A-Za-z_]+)\s*:\s*(.*)$/);
    if (!pair) continue;

    const key = pair[1].toLowerCase();
    const value = pair[2].trim();

    if (key === "version") {
      config.version = unquote(value);
    } else if (key === "reason") {
      config.reason = unquote(value) || DEFAULT_REASON;
    } else if (key === "scope") {
      const scope = parseList(value);
      if (scope.length > 0) config.scope = scope;
    } else if (key === "min_approvals") {
      const parsed = Number.parseInt(unquote(value), 10);
      if (Number.isFinite(parsed) && parsed >= 1) config.minApprovals = parsed;
    } else if (key === "effective_at") {
      const parsed = Date.parse(unquote(value));
      if (Number.isFinite(parsed)) config.effectiveAt = new Date(parsed).toISOString();
    }
  }

  return config;
}

function freezeAppliesTo(config, baseRef) {
  const base = normalizeBranch(baseRef);
  if (!base) return false;
  return config.scope.some((entry) => entry === "*" || entry === base);
}

function freezeTitle(version) {
  const label = String(version || "").trim();
  return label
    ? `Merge freeze: pre-${label} release`
    : "Merge freeze in effect";
}

function renderFreezeBody({
  version,
  reason,
  scope,
  minApprovals,
  effectiveAt,
  startedBy,
  startedAt,
}) {
  const scopeList = (scope && scope.length ? scope : DEFAULT_SCOPE).map(
    normalizeBranch,
  );
  const approvals = minApprovals || DEFAULT_MIN_APPROVALS;
  const versionLine = String(version || "").trim();

  return `## Merge freeze is in effect

${reason || DEFAULT_REASON}${versionLine ? ` (${versionLine})` : ""}

Pull requests targeting ${scopeList.map((b) => `\`${b}\``).join(", ")} cannot be
merged while this issue is open: the \`merge-freeze-gate\` check fails on them
and the merge queue rejects them.

Pull requests that were already in the merge queue when the freeze started are
allowed to finish; the freeze applies to everything queued afterwards.

### Landing something during the freeze

Genuinely urgent changes are still mergeable:

1. add the \`${EXEMPT_LABEL}\` label to the pull request,
2. collect **${approvals} approving reviews** from different people with write
   access, submitted against the pull request's current head commit.

Every exempted merge is recorded in this issue as an audit trail.

### Configuration

Edit the block below to re-scope the freeze or change how many approvals an
exemption needs. The gate re-reads it on every run, so changes take effect
immediately.

\`\`\`yaml
version: "${versionLine}"
reason: "${reason || DEFAULT_REASON}"
scope: [${scopeList.join(", ")}]
min_approvals: ${approvals}
effective_at: "${effectiveAt || ""}"
\`\`\`

\`effective_at\` is what lets the merge queue drain: entries created before that
moment are waved through. Do not move it forward by hand unless you mean to
re-open that window.

<sub>Started by @${startedBy || "unknown"}${startedAt ? ` on ${startedAt}` : ""}. Close this issue (or run the \`Merge freeze\` workflow with \`thaw\`) to lift the freeze.</sub>`;
}

function auditMarker(prNumber) {
  return `${AUDIT_MARKER_PREFIX}${prNumber} -->`;
}

// Oldest open `merge-freeze` issue wins, so a stray duplicate cannot take over.
async function findFreezeIssue(github, context) {
  const { owner, repo } = context.repo;
  const issues = await github.paginate(github.rest.issues.listForRepo, {
    owner,
    repo,
    state: "open",
    labels: FREEZE_LABEL,
    sort: "created",
    direction: "asc",
    per_page: 100,
  });

  return issues.find((issue) => !issue.pull_request) || null;
}

module.exports = {
  FREEZE_LABEL,
  EXEMPT_LABEL,
  DEFAULT_SCOPE,
  DEFAULT_MIN_APPROVALS,
  DEFAULT_REASON,
  auditMarker,
  findFreezeIssue,
  freezeAppliesTo,
  freezeTitle,
  normalizeBranch,
  parseFreezeConfig,
  renderFreezeBody,
};
