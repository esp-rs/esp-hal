const BUG_JURY_LABEL = "Bug Jury";
const TRACKER_LABEL = "backlog-bot-do-not-remove";
const DEFAULT_MIN_AGE_DAYS = 30;

function isOlderThanDays(issue, minAgeDays) {
  const createdAt = Date.parse(issue.created_at);
  if (Number.isNaN(createdAt)) return false;
  const minAgeMs = minAgeDays * 24 * 60 * 60 * 1000;
  return Date.now() - createdAt >= minAgeMs;
}

async function run({ github, context, core, batchSize, dryRun, minAgeDays }) {
  const owner = context.repo.owner;
  const repo = context.repo.repo;
  const ageDays = Number.isFinite(minAgeDays) && minAgeDays > 0 ? minAgeDays : DEFAULT_MIN_AGE_DAYS;

  core.info(
    `Starting bug-jury-labeler (dryRun=${dryRun}, batchSize=${batchSize}, minAgeDays=${ageDays})`,
  );

  const allIssues = [];
  let skippedTooNew = 0;
  for await (const { data: issues } of github.paginate.iterator(
    github.rest.issues.listForRepo,
    {
      owner,
      repo,
      state: "open",
      assignee: "none",
      sort: "created",
      direction: "asc",
      per_page: 100,
    },
  )) {
    for (const issue of issues) {
      if (issue.pull_request) continue;
      if (!isOlderThanDays(issue, ageDays)) {
        skippedTooNew++;
        continue;
      }
      allIssues.push(issue);
    }
  }

  core.info(`Open unassigned issues skipped (younger than ${ageDays} days): ${skippedTooNew}`);
  core.info(`Open unassigned issues older than ${ageDays} days: ${allIssues.length}`);

  if (allIssues.length === 0) {
    core.info(`No unassigned issues older than ${ageDays} days. Nothing to do.`);
    return;
  }

  const alreadyProcessed = [];
  let candidates = [];

  for (const issue of allIssues) {
    if (issue.labels.some((l) => l.name === TRACKER_LABEL)) {
      alreadyProcessed.push(issue);
    } else {
      candidates.push(issue);
    }
  }

  core.info(`Already processed (have "${TRACKER_LABEL}"): ${alreadyProcessed.length}`);
  core.info(`Candidates (no tracker label): ${candidates.length}`);

  let cycleRestarted = false;

  if (candidates.length === 0 && alreadyProcessed.length > 0) {
    core.info("All unassigned issues have been processed. Restarting cycle.");
    cycleRestarted = true;

    for (const issue of alreadyProcessed) {
      if (dryRun) {
        core.info(
          `[DRY RUN] #${issue.number}: Would remove "${TRACKER_LABEL}" label (cycle restart)`,
        );
      } else {
        await github.rest.issues.removeLabel({
          owner,
          repo,
          issue_number: issue.number,
          name: TRACKER_LABEL,
        });
        core.info(`#${issue.number}: Removed "${TRACKER_LABEL}" label (cycle restart)`);
      }
    }

    candidates = alreadyProcessed;
  }

  const batch = candidates.slice(0, batchSize);

  if (batch.length === 0) {
    core.info("No issues to label.");
    return;
  }

  let labeled = 0;
  for (const issue of batch) {
    if (dryRun) {
      core.info(
        `[DRY RUN] #${issue.number}: Would add "${BUG_JURY_LABEL}" + "${TRACKER_LABEL}" labels`,
      );
    } else {
      await github.rest.issues.addLabels({
        owner,
        repo,
        issue_number: issue.number,
        labels: [BUG_JURY_LABEL, TRACKER_LABEL],
      });
      core.info(`#${issue.number}: Added "${BUG_JURY_LABEL}" + "${TRACKER_LABEL}" labels`);
    }
    labeled++;
  }

  core.info("--- Summary ---");
  core.info(`Total unassigned issues: ${allIssues.length}`);
  core.info(`Already processed: ${alreadyProcessed.length}`);
  core.info(`Newly labeled: ${labeled}`);
  core.info(`Cycle restarted: ${cycleRestarted}`);
  core.info(`Remaining unlabeled after this run: ${candidates.length - labeled}`);
}

module.exports = { run };
