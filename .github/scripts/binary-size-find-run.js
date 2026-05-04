async function findBinarySizeRun({
  github,
  context,
  pr,
  example,
  chips,
  attempts = 10,
  delayMs = 3000,
}) {
  const { owner, repo } = context.repo;
  const expectedTitle = `Binary size for PR #${pr} (${example}; chips: ${chips})`;
  const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
  let run = null;

  for (let attempt = 1; attempt <= attempts; attempt++) {
    const { data } = await github.rest.actions.listWorkflowRuns({
      owner,
      repo,
      workflow_id: 'binary-size.yml',
      event: 'workflow_dispatch',
      per_page: 50,
    });

    if (data.workflow_runs && data.workflow_runs.length > 0) {
      const candidate = data.workflow_runs.find((r) =>
        (r.display_title && r.display_title.trim() === expectedTitle) ||
        (r.name && r.name.trim() === expectedTitle)
      );

      if (candidate) {
        run = candidate;
      }
    }

    await delay(delayMs);
  }

  let body = `Triggered **binary size analysis** for #${pr} (` +
    `example: \`${example}\`, chips: \`${chips}\`).`;

  if (run?.html_url) {
    body += `\n\nRun: ${run.html_url}`;
  } else {
    body += `\n\nCould not determine run URL yet. Please check the **Binary Size Analysis** workflow runs manually.`;
  }

  return { runId: run?.id ? String(run.id) : '', body };
}

module.exports = { findBinarySizeRun };
