// Required status check that blocks merges while a merge freeze is active.
//
// Re-runs on label and review changes, and again when the merge queue picks the
// pull request up.

const {
  EXEMPT_LABEL,
  auditMarker,
  findFreezeIssue,
  freezeAppliesTo,
  normalizeBranch,
  parseFreezeConfig,
} = require("./merge-freeze-config.js");

const WRITE_PERMISSIONS = new Set(["write", "maintain", "admin"]);

function labelNames(labels) {
  return (labels || []).map((label) =>
    typeof label === "string" ? label : label.name,
  );
}

// Reviews accumulate, so only each person's most recent one counts.
function latestReviewPerLogin(reviews) {
  const latest = new Map();

  for (const review of reviews || []) {
    const login = review && review.user && review.user.login;
    if (!login) continue;
    // Comments carry no verdict and must not displace an approval.
    if (review.state === "COMMENTED" || review.state === "PENDING") continue;

    const previous = latest.get(login);
    if (!previous || Number(review.id) > Number(previous.id)) {
      latest.set(login, review);
    }
  }

  return latest;
}

// Approvals must target the current head commit: an exemption covers the change
// that was actually reviewed, not whatever is pushed afterwards.
function classifyApprovals({ reviews, headSha, hasWriteAccess }) {
  const approved = [];
  const stale = [];
  const withoutWriteAccess = [];

  for (const [login, review] of latestReviewPerLogin(reviews)) {
    if (review.state !== "APPROVED") continue;

    if (review.commit_id !== headSha) {
      stale.push(login);
      continue;
    }
    if (!hasWriteAccess(login)) {
      withoutWriteAccess.push(login);
      continue;
    }

    approved.push(login);
  }

  return {
    approved: approved.sort(),
    stale: stale.sort(),
    withoutWriteAccess: withoutWriteAccess.sort(),
  };
}

// The merge commit's timestamp is when the pull request entered the queue.
// Entries older than the freeze are waved through so the queue can drain.
function queuedBeforeFreeze({ mergeGroup, effectiveAt }) {
  if (!mergeGroup || !effectiveAt) return false;

  const commit = mergeGroup.head_commit || {};
  const queuedAt = Date.parse(
    commit.timestamp || (commit.committer && commit.committer.date) || "",
  );
  const freezeAt = Date.parse(effectiveAt);
  if (!Number.isFinite(queuedAt) || !Number.isFinite(freezeAt)) return false;

  return queuedAt < freezeAt;
}

function evaluateFreezeGate({
  freeze,
  baseRef,
  labels,
  approvals,
  isDraft = false,
  queuedBeforeFreeze: alreadyQueued = false,
  exemptLabel = EXEMPT_LABEL,
}) {
  if (!freeze) {
    return { pass: true, code: "no-freeze", message: "No merge freeze active." };
  }

  const { config, issueNumber } = freeze;

  if (alreadyQueued) {
    return {
      pass: true,
      code: "queued-before-freeze",
      message: `Merge freeze (#${issueNumber}) is active, but this merge queue entry predates it and is allowed to finish.`,
    };
  }

  // Drafts cannot be merged anyway; `ready_for_review` re-evaluates them.
  if (isDraft) {
    return {
      pass: true,
      code: "draft",
      message: `Merge freeze (#${issueNumber}) is active; this draft is not blocked until it is marked ready for review.`,
    };
  }

  if (!freezeAppliesTo(config, baseRef)) {
    return {
      pass: true,
      code: "out-of-scope",
      message: `Merge freeze (#${issueNumber}) does not cover \`${normalizeBranch(baseRef)}\`.`,
    };
  }

  const freezeRef = `merge freeze (#${issueNumber})`;

  if (!labelNames(labels).includes(exemptLabel)) {
    return {
      pass: false,
      code: "missing-exempt-label",
      message: `Blocked by ${freezeRef}: ${config.reason}. If this change is urgent enough to land during the freeze, add the \`${exemptLabel}\` label and collect ${config.minApprovals} approving reviews from people with write access.`,
    };
  }

  const approved = approvals.approved || [];
  if (approved.length < config.minApprovals) {
    const details = [];
    if ((approvals.stale || []).length > 0) {
      details.push(
        `Not counted because they predate the current head commit: ${approvals.stale.map((l) => `@${l}`).join(", ")}.`,
      );
    }
    if ((approvals.withoutWriteAccess || []).length > 0) {
      details.push(
        `Not counted because they lack write access: ${approvals.withoutWriteAccess.map((l) => `@${l}`).join(", ")}.`,
      );
    }

    return {
      pass: false,
      code: "insufficient-approvals",
      message: `Exemption from ${freezeRef} requested, but only ${approved.length} of ${config.minApprovals} required approvals are in place.${details.length ? ` ${details.join(" ")}` : ""}`,
    };
  }

  return {
    pass: true,
    code: "exempt-approved",
    message: `Exempted from ${freezeRef}, approved by ${approved.map((l) => `@${l}`).join(", ")}.`,
  };
}

function pullRequestNumberFromMergeGroup(mergeGroup) {
  if (!mergeGroup) return 0;

  // `gh-readonly-queue/main/pr-1234-<sha>`: the queue's own naming is more
  // dependable than the commit message.
  const fromRef = String(mergeGroup.head_ref || "").match(/\/pr-(\d+)-/);
  if (fromRef) return Number(fromRef[1]);

  const firstLine = String(
    (mergeGroup.head_commit && mergeGroup.head_commit.message) || "",
  ).split("\n")[0];
  const fromMessage = firstLine.match(/#(\d+)/g);
  if (fromMessage && fromMessage.length > 0) {
    return Number(fromMessage[fromMessage.length - 1].slice(1));
  }

  return 0;
}

async function resolvePullRequest({ github, context, core }) {
  const { owner, repo } = context.repo;
  const payload = context.payload || {};

  if (payload.pull_request) {
    return payload.pull_request;
  }

  const number = pullRequestNumberFromMergeGroup(payload.merge_group);
  if (!number) {
    core.warning(
      "Could not determine the pull request behind this merge group entry.",
    );
    return null;
  }

  const { data } = await github.rest.pulls.get({
    owner,
    repo,
    pull_number: number,
  });
  return data;
}

function writeAccessChecker({ github, context }) {
  const { owner, repo } = context.repo;
  const cache = new Map();

  return async function hasWriteAccess(login) {
    if (cache.has(login)) return cache.get(login);

    let allowed = false;
    try {
      const { data } = await github.rest.repos.getCollaboratorPermissionLevel({
        owner,
        repo,
        username: login,
      });
      allowed = WRITE_PERMISSIONS.has(data.permission);
    } catch {
      // 404 for non-collaborators; any failure means the approval is not counted.
      allowed = false;
    }

    cache.set(login, allowed);
    return allowed;
  };
}

// Audit trail on the freeze issue. Best effort: fork pull requests get a
// read-only token, and the merge queue run records them instead.
async function recordExemption({ github, context, core, pull, issueNumber, verdict }) {
  const { owner, repo } = context.repo;
  const marker = auditMarker(pull.number);
  const body = `${marker}\n#${pull.number} merged during the freeze. ${verdict.message}`;

  try {
    const comments = await github.paginate(
      github.rest.issues.listComments,
      { owner, repo, issue_number: issueNumber, per_page: 100 },
    );
    const existing = comments.find((comment) =>
      String(comment.body || "").includes(marker),
    );

    if (existing) {
      await github.rest.issues.updateComment({
        owner,
        repo,
        comment_id: existing.id,
        body,
      });
    } else {
      await github.rest.issues.createComment({
        owner,
        repo,
        issue_number: issueNumber,
        body,
      });
    }
  } catch (error) {
    core.warning(`Could not record the exemption on #${issueNumber}: ${error.message}`);
  }
}

async function run({ github, context, core }) {
  const { owner, repo } = context.repo;
  const issue = await findFreezeIssue(github, context);

  if (!issue) {
    core.info("No merge freeze active.");
    await core.summary
      .addHeading("Merge freeze", 3)
      .addRaw("No merge freeze is active.")
      .write();
    return;
  }

  const config = parseFreezeConfig(issue.body);
  const alreadyQueued = queuedBeforeFreeze({
    mergeGroup: (context.payload || {}).merge_group,
    effectiveAt: config.effectiveAt,
  });

  if (alreadyQueued) {
    const verdict = evaluateFreezeGate({
      freeze: { config, issueNumber: issue.number },
      queuedBeforeFreeze: true,
    });
    core.info(verdict.message);
    await core.summary
      .addHeading("Merge freeze", 3)
      .addRaw(verdict.message)
      .addBreak()
      .addLink(`Freeze issue #${issue.number}`, issue.html_url)
      .write();
    return;
  }

  const pull = await resolvePullRequest({ github, context, core });

  if (!pull) {
    core.setFailed(
      `Merge freeze (#${issue.number}) is active and this run's pull request could not be identified.`,
    );
    return;
  }

  const baseRef = pull.base && pull.base.ref;
  let approvals = { approved: [], stale: [], withoutWriteAccess: [] };

  if (
    !pull.draft &&
    freezeAppliesTo(config, baseRef) &&
    labelNames(pull.labels).includes(EXEMPT_LABEL)
  ) {
    const reviews = await github.paginate(github.rest.pulls.listReviews, {
      owner,
      repo,
      pull_number: pull.number,
      per_page: 100,
    });

    const hasWriteAccess = writeAccessChecker({ github, context });
    const candidates = [...latestReviewPerLogin(reviews).values()].filter(
      (review) => review.state === "APPROVED",
    );
    const permissions = new Map();
    for (const review of candidates) {
      permissions.set(
        review.user.login,
        await hasWriteAccess(review.user.login),
      );
    }

    approvals = classifyApprovals({
      reviews,
      headSha: pull.head.sha,
      hasWriteAccess: (login) => permissions.get(login) === true,
    });
  }

  const verdict = evaluateFreezeGate({
    freeze: { config, issueNumber: issue.number },
    baseRef,
    labels: pull.labels,
    approvals,
    isDraft: Boolean(pull.draft),
  });

  await core.summary
    .addHeading("Merge freeze", 3)
    .addRaw(verdict.message)
    .addBreak()
    .addLink(`Freeze issue #${issue.number}`, issue.html_url)
    .write();

  if (!verdict.pass) {
    core.setFailed(verdict.message);
    return;
  }

  core.info(verdict.message);

  if (verdict.code === "exempt-approved") {
    await recordExemption({
      github,
      context,
      core,
      pull,
      issueNumber: issue.number,
      verdict,
    });
  }
}

module.exports = { run };
