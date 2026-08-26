// Starts and lifts a merge freeze.
//
// Freezing pins the freeze issue the gate reads and labels the pull requests it
// affects; thawing undoes both.

const {
  EXEMPT_LABEL,
  FREEZE_LABEL,
  DEFAULT_MIN_APPROVALS,
  DEFAULT_REASON,
  DEFAULT_SCOPE,
  findFreezeIssue,
  freezeAppliesTo,
  freezeTitle,
  normalizeBranch,
  parseFreezeConfig,
  renderFreezeBody,
} = require("./merge-freeze-config.js");

const LABEL_DEFINITIONS = [
  {
    name: FREEZE_LABEL,
    color: "b60205",
    description: "Blocked by an active merge freeze",
  },
  {
    name: EXEMPT_LABEL,
    color: "0e8a16",
    description: "May be merged during a merge freeze, with extra approvals",
  },
];

function parseScopeInput(value) {
  const scope = String(value || "")
    .split(/[\s,]+/)
    .map(normalizeBranch)
    .filter((entry) => entry.length > 0);

  return scope.length > 0 ? scope : [...DEFAULT_SCOPE];
}

async function ensureLabels({ github, context, core }) {
  const { owner, repo } = context.repo;

  for (const label of LABEL_DEFINITIONS) {
    try {
      await github.rest.issues.getLabel({ owner, repo, name: label.name });
    } catch {
      try {
        await github.rest.issues.createLabel({ owner, repo, ...label });
        core.info(`Created label "${label.name}".`);
      } catch (error) {
        core.warning(`Could not create label "${label.name}": ${error.message}`);
      }
    }
  }
}

async function setPinned({ github, core, nodeId, pinned }) {
  const mutation = pinned
    ? `mutation($id: ID!) { pinIssue(input: { issueId: $id }) { issue { number } } }`
    : `mutation($id: ID!) { unpinIssue(input: { issueId: $id }) { issue { number } } }`;

  try {
    await github.graphql(mutation, { id: nodeId });
  } catch (error) {
    // At most three issues can be pinned, and that limit must not fail a freeze.
    core.warning(
      `Could not ${pinned ? "pin" : "unpin"} the freeze issue: ${error.message}`,
    );
  }
}

// Pull requests already in the queue are allowed to finish, so labelling them
// would advertise a block that does not apply to them.
async function queuedPullRequests({ github, context, core, scope }) {
  const { owner, repo } = context.repo;
  const query = `
    query($owner: String!, $repo: String!, $branch: String!) {
      repository(owner: $owner, name: $repo) {
        mergeQueue(branch: $branch) {
          entries(first: 100) {
            nodes { pullRequest { number } }
          }
        }
      }
    }`;

  const queued = new Set();

  for (const branch of scope) {
    try {
      const result = await github.graphql(query, { owner, repo, branch });
      const nodes =
        result.repository &&
        result.repository.mergeQueue &&
        result.repository.mergeQueue.entries
          ? result.repository.mergeQueue.entries.nodes || []
          : [];
      for (const node of nodes) {
        if (node && node.pullRequest) queued.add(node.pullRequest.number);
      }
    } catch (error) {
      core.warning(
        `Could not read the merge queue for \`${branch}\`: ${error.message}`,
      );
    }
  }

  return queued;
}

async function openPullRequestsInScope({ github, context, scope }) {
  const { owner, repo } = context.repo;
  const pulls = await github.paginate(github.rest.pulls.list, {
    owner,
    repo,
    state: "open",
    per_page: 100,
  });

  return pulls.filter((pull) =>
    freezeAppliesTo({ scope }, pull.base && pull.base.ref),
  );
}

async function labelPullRequests({ github, context, core, scope, add, skip }) {
  const { owner, repo } = context.repo;
  const pulls = await openPullRequestsInScope({ github, context, scope });
  const skipped = skip || new Set();
  let changed = 0;

  for (const pull of pulls) {
    if (skipped.has(pull.number)) continue;

    const labelled = (pull.labels || []).some(
      (label) => label.name === FREEZE_LABEL,
    );
    if (add === labelled) continue;

    try {
      if (add) {
        await github.rest.issues.addLabels({
          owner,
          repo,
          issue_number: pull.number,
          labels: [FREEZE_LABEL],
        });
      } else {
        await github.rest.issues.removeLabel({
          owner,
          repo,
          issue_number: pull.number,
          name: FREEZE_LABEL,
        });
      }
      changed += 1;
    } catch (error) {
      core.warning(
        `Could not update labels on #${pull.number}: ${error.message}`,
      );
    }
  }

  core.info(
    `${add ? "Labelled" : "Unlabelled"} ${changed} of ${pulls.length} open pull requests in scope.`,
  );
  return { changed, total: pulls.length };
}

async function freeze({ github, context, core, version, reason, scope, minApprovals }) {
  const { owner, repo } = context.repo;
  const startedBy = context.actor;
  const now = new Date().toISOString();

  await ensureLabels({ github, context, core });

  const existing = await findFreezeIssue(github, context);
  // Keep the original moment: adjusting a freeze must not re-open the queue
  // grace window.
  const effectiveAt = existing
    ? parseFreezeConfig(existing.body).effectiveAt || now
    : now;
  const body = renderFreezeBody({
    version,
    reason,
    scope,
    minApprovals,
    effectiveAt,
    startedBy,
    startedAt: now.slice(0, 10),
  });

  let issue;

  if (existing) {
    // Re-running `freeze` adjusts a live freeze, so update instead of opening a
    // second issue.
    const { data } = await github.rest.issues.update({
      owner,
      repo,
      issue_number: existing.number,
      title: freezeTitle(version),
      body,
    });
    issue = data;
    core.info(`Updated the existing freeze issue #${issue.number}.`);
  } else {
    const { data } = await github.rest.issues.create({
      owner,
      repo,
      title: freezeTitle(version),
      body,
      labels: [FREEZE_LABEL],
    });
    issue = data;
    core.info(`Opened freeze issue #${issue.number}.`);
  }

  await setPinned({ github, core, nodeId: issue.node_id, pinned: true });

  const queued = await queuedPullRequests({ github, context, core, scope });
  if (queued.size > 0) {
    core.info(
      `Left ${queued.size} pull request(s) already in the merge queue to finish: ${[...queued]
        .map((number) => `#${number}`)
        .join(", ")}.`,
    );
  }

  const labelled = await labelPullRequests({
    github,
    context,
    core,
    scope,
    add: true,
    skip: queued,
  });

  const summary = core.summary
    .addHeading("Merge freeze started", 3)
    .addRaw(`${reason}${version ? ` (${version})` : ""}`)
    .addBreak()
    .addRaw(`Scope: ${scope.join(", ")} · approvals required for an exemption: ${minApprovals}`)
    .addBreak()
    .addRaw(`Effective from ${effectiveAt}; merge queue entries created before that finish normally.`)
    .addBreak()
    .addRaw(`Labelled ${labelled.changed} of ${labelled.total} affected pull requests.`);

  if (queued.size > 0) {
    summary
      .addBreak()
      .addRaw(
        `Draining the merge queue: ${[...queued].map((number) => `#${number}`).join(", ")}.`,
      );
  }

  await summary
    .addBreak()
    .addLink(`Freeze issue #${issue.number}`, issue.html_url)
    .write();
}

async function thaw({ github, context, core }) {
  const { owner, repo } = context.repo;
  const issue = await findFreezeIssue(github, context);

  if (!issue) {
    core.info("No merge freeze is active; nothing to lift.");
    await core.summary
      .addHeading("Merge freeze", 3)
      .addRaw("No merge freeze was active.")
      .write();
    return;
  }

  const config = parseFreezeConfig(issue.body);

  const labelled = await labelPullRequests({
    github,
    context,
    core,
    scope: config.scope,
    add: false,
  });

  await setPinned({ github, core, nodeId: issue.node_id, pinned: false });

  await github.rest.issues.createComment({
    owner,
    repo,
    issue_number: issue.number,
    body: `Merge freeze lifted by @${context.actor}. Merges into ${config.scope
      .map((branch) => `\`${branch}\``)
      .join(", ")} are open again.`,
  });

  await github.rest.issues.update({
    owner,
    repo,
    issue_number: issue.number,
    state: "closed",
    state_reason: "completed",
  });

  core.info(`Closed freeze issue #${issue.number}.`);
  await core.summary
    .addHeading("Merge freeze lifted", 3)
    .addRaw(`Unlabelled ${labelled.changed} of ${labelled.total} pull requests.`)
    .addBreak()
    .addLink(`Freeze issue #${issue.number}`, issue.html_url)
    .write();
}

async function run({ github, context, core, state, version, reason, scope, minApprovals }) {
  const requestedState = String(state || "").toLowerCase();

  if (requestedState === "thaw") {
    await thaw({ github, context, core });
    return;
  }
  if (requestedState !== "freeze") {
    core.setFailed(`Unknown state "${state}"; expected "freeze" or "thaw".`);
    return;
  }

  const parsedApprovals = Number.parseInt(minApprovals, 10);

  await freeze({
    github,
    context,
    core,
    version: String(version || "").trim(),
    reason: String(reason || "").trim() || DEFAULT_REASON,
    scope: parseScopeInput(scope),
    minApprovals:
      Number.isFinite(parsedApprovals) && parsedApprovals >= 1
        ? parsedApprovals
        : DEFAULT_MIN_APPROVALS,
  });
}

module.exports = { run };
