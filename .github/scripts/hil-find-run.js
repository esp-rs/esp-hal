async function findHilRun({
  github,
  context,
  pr,
  selector,
  tests = '',
  attempts = 10,
  delayMs = 3000,
}) {
  const { owner, repo } = context.repo;
  const tests_trimmed = String(tests || '').trim();

  const expectedTitle = tests_trimmed
    ? `HIL for PR #${pr} (${selector}; tests: ${tests_trimmed})`
    : `HIL for PR #${pr} (${selector})`;

  const delay = (ms) => new Promise((r) => setTimeout(r, ms));
  let run = null;

  // Poll multiple times; keep the *last* matching run we see
  for (let attempt = 1; attempt <= attempts; attempt++) {
    const { data } = await github.rest.actions.listWorkflowRuns({
      owner,
      repo,
      workflow_id: 'hil.yml',
      event: 'workflow_dispatch',
      per_page: 50,
    });

    if (data.workflow_runs && data.workflow_runs.length > 0) {
      const candidate = data.workflow_runs.find(r =>
        (r.display_title && r.display_title.trim() === expectedTitle) ||
        (r.name && r.name.trim() === expectedTitle)
      );
      // Keep the *last* matching run we see across all attempts
      if (candidate) {
        run = candidate;
      }
    }
    await delay(delayMs);
  }

  const isMatrix = selector === 'quick' || selector === 'full';

  let body = isMatrix
    ? `Triggered **${selector}** HIL run for #${pr}.`
    : `Triggered **HIL** run for #${pr} (chips: ${selector}).`;

  if (run?.html_url) {
    body += `\n\nRun: ${run.html_url}`;
  } else {
    body += `\n\nCould not determine run URL yet. Please check the **HIL** workflow runs manually.`;
  }

  return { runId: run?.id ? String(run.id) : '', body };
}

module.exports = { findHilRun };
