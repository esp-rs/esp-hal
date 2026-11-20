const RUN_TESTS_STEP = "Run Tests";

function isHilRunMatrixJob(name) {
  // Matches "hil-run (…)" but not "hil-run-radio (…)".
  return /(?:^|\/\s*)hil-run \(/i.test(String(name || ""));
}

function classifyMatrixJob(job) {
  // GitHub names each leg "hil-run (<soc>, <arch>, <runner>)".
  const soc = /hil-run \(([^,)]+)/i.exec(job.name)?.[1];
  const steps = job.steps || [];
  const runTests = steps.find((s) => s.name === RUN_TESTS_STEP);
  if (!runTests) {
    return { error: `job "${job.name}" is missing "${RUN_TESTS_STEP}" step` };
  }

  const conclusion = runTests.conclusion;
  if (conclusion === "skipped") {
    // The guard step skips a chip that produced no ELFs. A job that failed, was
    // cancelled or timed out leaves the same skipped step behind, so only a
    // successful job means "not tested".
    if (job.conclusion === "success") {
      return { soc, kind: "skipped" };
    }
    return { soc, kind: "failed" };
  }
  if (conclusion === "success") {
    return { soc, kind: "passed" };
  }
  if (
    conclusion === "failure" ||
    conclusion === "cancelled" ||
    conclusion === null
  ) {
    return { soc, kind: "failed" };
  }

  return {
    error: `job "${job.name}" has unexpected "${RUN_TESTS_STEP}" conclusion: ${conclusion}`,
  };
}

function evaluateHilRunResults(classifications, requiredChips = []) {
  const executed = classifications.filter(
    (c) => c.kind === "passed" || c.kind === "failed",
  );
  const failures = classifications.filter((c) => c.kind === "failed");
  // A required chip must have passed, so a mistyped label fails too.
  const requiredNotPassed = requiredChips.filter(
    (soc) => !classifications.some((c) => c.soc === soc && c.kind === "passed"),
  );

  const pass = failures.length < 2 && requiredNotPassed.length === 0;
  return {
    pass,
    executed: executed.length,
    failures: failures.length,
    requiredNotPassed,
  };
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

    const page_jobs = data.jobs || [];
    jobs.push(...page_jobs);
    // An empty page ends the walk even when total_count claims more: the count
    // can drift while the run is still going, and trusting it alone spins here
    // until the job times out.
    if (page_jobs.length === 0 || jobs.length >= data.total_count) {
      break;
    }
    page += 1;
  }

  return jobs;
}

async function evaluateHilGate({ github, context, core, requiredChips = "" }) {
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

  const verdict = evaluateHilRunResults(
    classifications,
    requiredChips.split(" ").filter(Boolean),
  );
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

  if (verdict.requiredNotPassed.length > 0) {
    core.setFailed(
      `HIL gate failed: required chips did not pass: ${verdict.requiredNotPassed.join(", ")}`,
    );
    return;
  }
  core.setFailed(
    `HIL gate failed: ${verdict.failures}/${verdict.executed} executed runners failed`,
  );
}

module.exports = {
  RUN_TESTS_STEP,
  isHilRunMatrixJob,
  classifyMatrixJob,
  evaluateHilRunResults,
  evaluateHilGate,
};
