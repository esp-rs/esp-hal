const RUN_TESTS_STEP = "Run Tests";

function isHilRunMatrixJob(name) {
  // Matches "hil-run (…)" but not "hil-run-radio (…)".
  return /^hil-run \(/i.test(String(name || ""));
}

function classifyMatrixJob(job) {
  const steps = job.steps || [];
  const runTests = steps.find((s) => s.name === RUN_TESTS_STEP);
  if (!runTests) {
    return { error: `job "${job.name}" is missing "${RUN_TESTS_STEP}" step` };
  }

  const conclusion = runTests.conclusion;
  if (conclusion === "skipped") {
    return { kind: "skipped" };
  }
  if (conclusion === "success") {
    return { kind: "passed" };
  }
  if (
    conclusion === "failure" ||
    conclusion === "cancelled" ||
    conclusion === null
  ) {
    return { kind: "failed" };
  }

  return {
    error: `job "${job.name}" has unexpected "${RUN_TESTS_STEP}" conclusion: ${conclusion}`,
  };
}

function evaluateHilRunResults(classifications) {
  const executed = classifications.filter(
    (c) => c.kind === "passed" || c.kind === "failed",
  );
  const failures = classifications.filter((c) => c.kind === "failed");

  if (executed.length === 0) {
    return { pass: true, executed: 0, failures: 0 };
  }

  const pass = failures.length * 2 < executed.length;
  return { pass, executed: executed.length, failures: failures.length };
}

async function listWorkflowRunJobs(github, context) {
  const { owner, repo } = context.repo;
  const run_id = context.runId;
  const jobs = [];
  let page = 1;

  while (true) {
    const { data } = await github.rest.actions.listJobsForWorkflowRun({
      owner,
      repo,
      run_id,
      per_page: 100,
      page,
    });

    jobs.push(...(data.jobs || []));
    if (jobs.length >= data.total_count) {
      break;
    }
    page += 1;
  }

  return jobs;
}

async function evaluateHilGate({ github, context, core }) {
  const jobs = await listWorkflowRunJobs(github, context);
  const matrixJobs = jobs.filter((job) => isHilRunMatrixJob(job.name));

  if (matrixJobs.length === 0) {
    core.setFailed("HIL gate failed: could not find hil-run matrix jobs");
    return;
  }

  const classifications = [];
  for (const job of matrixJobs) {
    const result = classifyMatrixJob(job);
    if (result.error) {
      core.setFailed(`HIL gate failed: ${result.error}`);
      return;
    }
    classifications.push(result);
  }

  const verdict = evaluateHilRunResults(classifications);
  if (verdict.pass) {
    if (verdict.executed > 0) {
      core.info(
        `HIL gate passed (${verdict.failures}/${verdict.executed} executed runners failed)`,
      );
    } else {
      core.info("HIL gate passed (no hil-run matrix legs executed tests)");
    }
    return;
  }

  core.setFailed("HIL gate failed: ≥50% of executed runners failed");
}

module.exports = {
  RUN_TESTS_STEP,
  isHilRunMatrixJob,
  classifyMatrixJob,
  evaluateHilRunResults,
  evaluateHilGate,
};
