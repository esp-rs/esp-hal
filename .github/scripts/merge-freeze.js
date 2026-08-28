// Merge freeze: `toggle` starts and lifts it, `gate` enforces it, and
// `recordExemptMerge` notes what slipped through.
//
// The state is an open issue labelled `merge-freeze`. While it is open, merges
// into `main` are rejected unless the pull request carries
// `merge-freeze-exempt`.

// We only ever hold merging into `main`, so the freeze is not configurable.
// This can be customized in future if we want to freeze other branches.
const FROZEN_BRANCH = "main";

const FREEZE_LABEL = "merge-freeze";
const EXEMPT_LABEL = "merge-freeze-exempt";
const NOTICE_MARKER = "<!-- merge-freeze-notice -->";
const auditMarker = (prNumber) => `<!-- merge-freeze-audit:${prNumber} -->`;

// The gate is required for every merge, so a transient API error would block all
// merging. Retry before giving up. A failure still fails closed.
async function withRetries({ core, attempts = 3, delayMs = 2000 }, action) {
  for (let attempt = 1; ; attempt += 1) {
    try {
      return await action();
    } catch (error) {
      // A 404 or a 422 will not become a different answer on the second ask.
      const permanent =
        error.status >= 400 && error.status < 500 && error.status !== 429;
      if (permanent || attempt >= attempts) throw error;

      core.warning(
        `${error.message} - retrying (attempt ${attempt} of ${attempts}).`,
      );
      await new Promise((resolve) => setTimeout(resolve, delayMs * attempt));
    }
  }
}

// The label can be applied by hand, so more than one issue may claim to hold the
// freeze. Everything works off this list, oldest first.
async function findFreezeIssues({ github, context }) {
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

  return issues.filter((issue) => !issue.pull_request);
}

// Oldest open `merge-freeze` issue wins, so a stray duplicate cannot take over.
async function findFreezeIssue({ github, context }) {
  const issues = await findFreezeIssues({ github, context });
  return issues[0] || null;
}

function freezeTitle(version) {
  const label = String(version || "").trim();
  return label ? `Merge freeze: pre-${label} release` : "Merge freeze in effect";
}

function freezeBody({ reason, startedBy }) {
  return `## Merge freeze is in effect

${reason}

While this issue is open, the merge queue rejects pull requests targeting
\`${FROZEN_BRANCH}\`: CI's \`merge-freeze-gate\` job fails on them, which fails
\`ci-result\`. Review carries on as usual — nothing is blocked until an actual
merge attempt.

If a change is urgent enough to land during the freeze, add the
\`${EXEMPT_LABEL}\` label to it. Exempted merges are recorded below.

<sub>Started by @${startedBy}. Close this issue (or run the \`Merge freeze\` workflow with \`thaw\`) to lift the freeze.</sub>`;
}

async function upsertComment({
  github,
  context,
  core,
  issueNumber,
  marker,
  body,
  updateOnly = false,
}) {
  const { owner, repo } = context.repo;

  try {
    const comments = await github.paginate(github.rest.issues.listComments, {
      owner,
      repo,
      issue_number: issueNumber,
      per_page: 100,
    });
    const existing = comments.find((comment) =>
      String(comment.body || "").includes(marker),
    );

    if (existing) {
      await github.rest.issues.updateComment({
        owner,
        repo,
        comment_id: existing.id,
        body: `${marker}\n${body}`,
      });
    } else if (!updateOnly) {
      await github.rest.issues.createComment({
        owner,
        repo,
        issue_number: issueNumber,
        body: `${marker}\n${body}`,
      });
    }
  } catch (error) {
    core.warning(`Could not comment on #${issueNumber}: ${error.message}`);
  }
}

// A merge group can batch several pull requests, and its ref names only one of
// them, so walk the commits the group adds on top of the base as well.
async function pullRequestNumbersFromMergeGroup({ github, context, core }) {
  const { owner, repo } = context.repo;
  const mergeGroup = (context.payload || {}).merge_group;
  if (!mergeGroup) return [];

  const numbers = new Set();

  // `gh-readonly-queue/main/pr-1234-<sha>`.
  const fromRef = String(mergeGroup.head_ref || "").match(/\/pr-(\d+)-/);
  if (fromRef) numbers.add(Number(fromRef[1]));

  if (mergeGroup.base_sha && mergeGroup.head_sha) {
    try {
      const { data } = await withRetries({ core }, () =>
        github.rest.repos.compareCommitsWithBasehead({
          owner,
          repo,
          basehead: `${mergeGroup.base_sha}...${mergeGroup.head_sha}`,
        }),
      );

      for (const commit of data.commits || []) {
        // Only the `(#1234)` suffix of a merge or squash subject: a bare
        // `#1234` anywhere else is as likely to be an issue reference.
        const subject = String(commit.commit.message || "").split("\n")[0];
        const suffix = subject.match(/\(#(\d+)\)\s*$/);
        if (suffix) numbers.add(Number(suffix[1]));
      }
    } catch (error) {
      core.warning(
        `Could not list the commits of this merge queue entry: ${error.message}`,
      );
    }
  }

  return [...numbers].sort((a, b) => a - b);
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

async function gate({ github, context, core }) {
  const { owner, repo } = context.repo;
  let issue;

  try {
    issue = await withRetries({ core }, () =>
      findFreezeIssue({ github, context }),
    );
  } catch (error) {
    core.setFailed(
      `Could not determine whether a merge freeze is active: ${error.message}`,
    );
    return;
  }

  if (!issue) {
    core.info("No merge freeze active.");
    return;
  }

  const mergeGroup = (context.payload || {}).merge_group;
  if (!mergeGroup) {
    core.info(
      `Merge freeze (#${issue.number}) is active, and it is enforced when a pull request enters the merge queue.`,
    );
    return;
  }

  const baseRef = String(mergeGroup.base_ref || "").replace(/^refs\/heads\//, "");
  if (baseRef !== FROZEN_BRANCH) {
    core.info(
      `Merge freeze (#${issue.number}) only covers \`${FROZEN_BRANCH}\`, not \`${baseRef}\`.`,
    );
    return;
  }

  const numbers = await pullRequestNumbersFromMergeGroup({
    github,
    context,
    core,
  });

  const pulls = [];
  for (const number of numbers) {
    try {
      const response = await withRetries({ core }, () =>
        github.rest.pulls.get({ owner, repo, pull_number: number }),
      );
      pulls.push(response.data);
    } catch (error) {
      // A number scraped from a commit subject need not be a pull request at
      // all. As long as the entry yields one, the freeze can still be judged.
      core.warning(`Could not read #${number}: ${error.message}`);
    }
  }

  if (pulls.length === 0) {
    core.setFailed(
      `Merge freeze (#${issue.number}) is active and the pull requests behind this merge queue entry could not be identified.`,
    );
    return;
  }

  const exempt = (pull) =>
    (pull.labels || []).some((label) => label.name === EXEMPT_LABEL);
  const blocked = pulls.filter((pull) => !exempt(pull));
  const list = (items) => items.map((pull) => `#${pull.number}`).join(", ");

  await core.summary
    .addHeading("Merge freeze", 3)
    .addRaw(
      blocked.length === 0
        ? `${list(pulls)} carries \`${EXEMPT_LABEL}\` and may merge during the freeze.`
        : `${list(blocked)} is blocked by the merge freeze.`,
    )
    .addBreak()
    .addLink(`Freeze issue #${issue.number}`, issue.html_url)
    .write();

  // Clear earlier "blocked" notices so an exempted pull request does not keep
  // claiming it cannot be merged.
  for (const pull of pulls.filter(exempt)) {
    await upsertComment({
      github,
      context,
      core,
      issueNumber: pull.number,
      marker: NOTICE_MARKER,
      body: `\`${EXEMPT_LABEL}\` is set, so merge freeze (#${issue.number}) no longer blocks this pull request.`,
      updateOnly: true,
    });
  }

  if (blocked.length === 0) {
    core.info(`${list(pulls)} is exempt from merge freeze (#${issue.number}).`);
    return;
  }

  const message = `Blocked by merge freeze (#${issue.number}). If this change is urgent enough to land during the freeze, add the \`${EXEMPT_LABEL}\` label.`;

  // The merge attempt is the first time the author hears about the freeze, so
  // spell out the way forward on the pull request itself.
  for (const pull of blocked) {
    await upsertComment({
      github,
      context,
      core,
      issueNumber: pull.number,
      marker: NOTICE_MARKER,
      body: `${message}\n\nSee ${issue.html_url} for the details of this freeze.`,
    });
  }

  // A batched entry is only as mergeable as its least mergeable member.
  core.setFailed(
    pulls.length === 1
      ? message
      : `${list(blocked)} of this merge group is blocked by merge freeze (#${issue.number}).`,
  );
}

// The gate runs before the rest of CI, so it cannot claim anything was merged.
// This runs once the merge has actually happened.
async function recordExemptMerge({ github, context, core }) {
  const pull = (context.payload || {}).pull_request;
  if (!pull || !pull.merged) return;

  if (String((pull.base || {}).ref || "") !== FROZEN_BRANCH) return;
  if (!(pull.labels || []).some((label) => label.name === EXEMPT_LABEL)) return;

  const issue = await findFreezeIssue({ github, context });
  if (!issue) {
    core.info(
      `#${pull.number} was merged with \`${EXEMPT_LABEL}\`, but no freeze is active - nothing to record.`,
    );
    return;
  }

  await upsertComment({
    github,
    context,
    core,
    issueNumber: issue.number,
    marker: auditMarker(pull.number),
    body: `#${pull.number} (${pull.title}) was merged during the freeze, exempted by \`${EXEMPT_LABEL}\`.`,
  });
  core.info(`Recorded #${pull.number} on freeze issue #${issue.number}.`);
}

async function freeze({ github, context, core, version, reason }) {
  const { owner, repo } = context.repo;
  const title = freezeTitle(version);
  const body = freezeBody({ reason, startedBy: context.actor });
  const open = await findFreezeIssues({ github, context });
  const existing = open[0] || null;
  let issue;

  if (open.length > 1) {
    core.warning(
      `Several issues carry the \`${FREEZE_LABEL}\` label: ${open.map((i) => `#${i.number}`).join(", ")}. Updating the oldest; \`thaw\` will close all of them.`,
    );
  }

  if (existing) {
    const { data } = await github.rest.issues.update({
      owner,
      repo,
      issue_number: existing.number,
      title,
      body,
    });
    issue = data;
    core.info(`Updated the existing freeze issue #${issue.number}.`);
  } else {
    const { data } = await github.rest.issues.create({
      owner,
      repo,
      title,
      body,
      labels: [FREEZE_LABEL],
    });
    issue = data;
    core.info(`Opened freeze issue #${issue.number}.`);
  }

  await setPinned({ github, core, nodeId: issue.node_id, pinned: true });
  await core.summary
    .addHeading("Merge freeze started", 3)
    .addRaw(reason)
    .addBreak()
    .addLink(`Freeze issue #${issue.number}`, issue.html_url)
    .write();
}

async function thaw({ github, context, core }) {
  const { owner, repo } = context.repo;
  const open = await findFreezeIssues({ github, context });

  if (open.length === 0) {
    core.info("No merge freeze is active, so there is nothing to lift.");
    return;
  }

  // Every open one has to go: leaving a hand-labelled duplicate behind would
  // keep the queue closed while the workflow reports the freeze as lifted.
  for (const issue of open) {
    await setPinned({ github, core, nodeId: issue.node_id, pinned: false });
    await github.rest.issues.createComment({
      owner,
      repo,
      issue_number: issue.number,
      body: `Merge freeze lifted by @${context.actor}. Merges into \`${FROZEN_BRANCH}\` are open again.`,
    });
    await github.rest.issues.update({
      owner,
      repo,
      issue_number: issue.number,
      state: "closed",
      state_reason: "completed",
    });
    core.info(`Closed freeze issue #${issue.number}.`);
  }

  const summary = core.summary
    .addHeading("Merge freeze lifted", 3)
    .addRaw(`Merges into \`${FROZEN_BRANCH}\` are open again.`)
    .addBreak();
  for (const issue of open) {
    summary.addLink(`Freeze issue #${issue.number}`, issue.html_url).addBreak();
  }
  await summary.write();
}

async function toggle({ github, context, core, state, version, reason }) {
  const requested = String(state || "").toLowerCase();

  if (requested === "thaw") {
    await thaw({ github, context, core });
    return;
  }
  if (requested !== "freeze") {
    core.setFailed(`Unknown state "${state}"; expected "freeze" or "thaw".`);
    return;
  }

  await freeze({
    github,
    context,
    core,
    version: String(version || "").trim(),
    reason: String(reason || "").trim() || "pre-release merge freeze",
  });
}

module.exports = { gate, toggle, recordExemptMerge };
