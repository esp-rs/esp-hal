// Verdict reporting for dispatched HIL runs.
//
// hil.yml's report job calls this when a dispatched run finishes.
const STATUS_MARKER = "<!-- HIL_STATUS -->";

function statusText(conclusion, attempt) {
  const rerun = attempt > 1 ? ` on re-run (attempt ${attempt})` : "";
  if (conclusion === "success") {
    return `**Status update:** ✅ HIL run **succeeded**${rerun}.`;
  }
  if (conclusion === "cancelled") {
    return `**Status update:** ⚠️ HIL run was **cancelled**${rerun}.`;
  }
  return `**Status update:** ❌ HIL run **failed**${rerun} (conclusion: ${conclusion}).`;
}

function applyStatus(body, text) {
  const [head] = body.split(STATUS_MARKER);
  return `${head.trimEnd()}\n\n${STATUS_MARKER}\n${text}`;
}

function runConclusion(results) {
  if (results.includes("cancelled")) return "cancelled";
  if (results.includes("failure")) return "failure";
  return "success";
}

async function updateRunStatus({ github, context, core, pr, conclusion }) {
  const { owner, repo } = context.repo;
  const runId = context.runId;
  const attempt = Number(process.env.GITHUB_RUN_ATTEMPT);

  // The dispatcher's confirmation comment is the one carrying this run's URL.
  const comments = await github.paginate(github.rest.issues.listComments, {
    owner,
    repo,
    issue_number: pr,
    per_page: 100,
  });
  const comment = comments.findLast(
    (c) =>
      c.user?.login === "github-actions[bot]" &&
      c.body?.includes(`/actions/runs/${runId}`),
  );
  if (!comment) {
    core.info(`No bot comment on #${pr} references run ${runId}.`);
    return;
  }

  await github.rest.issues.updateComment({
    owner,
    repo,
    comment_id: comment.id,
    body: applyStatus(comment.body, statusText(conclusion, attempt)),
  });
}

module.exports = { updateRunStatus, runConclusion };
