// Correlating a workflow we just dispatched with its actual run.
//
// GitHub's `workflow_dispatch` REST API returns 204 No Content with **no run
// id**, so the dispatcher cannot learn which run it created from the API
// response. Matching on the run *title* alone is ambiguous: a previous dispatch
// for the same PR/chips produces an identical title, so an hour-old run can be
// returned as if it were the one we just triggered.
//
// To make the correlation exact, the dispatcher passes a unique `distinct_id`
// input, and the dispatched workflow embeds it into its `run-name` via the
// marker below. We then poll for the (single) run whose title contains that
// marker.
function dispatchMarker(distinctId) {
  const id = String(distinctId ?? '').trim();
  if (!id) {
    // An empty id would produce `[dispatch-id: ]`, which correlates nothing.
    // Fail loudly so a misconfigured dispatcher is obvious rather than silently
    // never matching.
    throw new Error('dispatchMarker: a non-empty distinctId is required');
  }
  return `[dispatch-id: ${id}]`;
}

// Poll the workflow's recent dispatch runs until one carries our marker.
async function findDispatchedRun({
  github,
  context,
  workflowId,
  distinctId,
  attempts = 20,
  delayMs = 3000,
}) {
  const { owner, repo } = context.repo;
  const marker = dispatchMarker(distinctId);
  const delay = (ms) => new Promise((r) => setTimeout(r, ms));

  for (let attempt = 1; attempt <= attempts; attempt++) {
    const { data } = await github.rest.actions.listWorkflowRuns({
      owner,
      repo,
      workflow_id: workflowId,
      event: 'workflow_dispatch',
      per_page: 50,
    });

    const run = (data.workflow_runs || []).find(
      (r) =>
        (r.display_title && r.display_title.includes(marker)) ||
        (r.name && r.name.includes(marker))
    );

    if (run) return run;

    // The run we just dispatched may not be visible yet; wait and retry.
    // Don't sleep after the final check.
    if (attempt < attempts) await delay(delayMs);
  }

  return null;
}

async function findHilRun({ github, context, pr, selector, distinctId }) {
  const run = await findDispatchedRun({
    github,
    context,
    workflowId: 'hil.yml',
    distinctId,
  });

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

module.exports = { findHilRun, findDispatchedRun, dispatchMarker };
