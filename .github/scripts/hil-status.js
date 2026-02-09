function statusSuffix(kind, conclusion) {
  if (!conclusion) {
    return `\n\n**Status update:** ${kind} run is still in progress or status unknown.`;
  }
  if (conclusion === "success") {
    return `\n\n**Status update:** ✅ ${kind} run **succeeded**.`;
  }
  if (conclusion === "cancelled") {
    return `\n\n**Status update:** ⚠️ ${kind} run was **cancelled**.`;
  }
  return `\n\n**Status update:** ❌ ${kind} run **failed** (conclusion: ${conclusion}).`;
}

async function pollRun({
  github,
  context,
  runId,
  commentId,
  kind,
  maxPolls = 60,
  pollIntervalMs = 15000,
}) {
  const { owner, repo } = context.repo;
  const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

  let conclusion = null;

  // Poll up to ~15 minutes by default (60 * 15s)
  for (let i = 0; i < maxPolls; i++) {
    const { data } = await github.rest.actions.getWorkflowRun({
      owner,
      repo,
      run_id: runId,
    });

    if (data.status === "completed") {
      conclusion = data.conclusion;
      break;
    }

    await delay(pollIntervalMs);
  }

  const suffix = statusSuffix(kind, conclusion);

  const comment = await github.rest.issues.getComment({
    owner,
    repo,
    comment_id: commentId,
  });

  const body = `${comment.data.body}\n${suffix}`;

  await github.rest.issues.updateComment({
    owner,
    repo,
    comment_id: commentId,
    body,
  });
}

module.exports = { pollRun };
