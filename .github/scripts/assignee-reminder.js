const INACTIVITY_DAYS = 45;
const MS_PER_DAY = 24 * 60 * 60 * 1000;
const REMINDER_MARKER = "<!-- assignee-reminder -->";
const COMMENT_TEMPLATE = `${REMINDER_MARKER}
Dear {mentions},

This is a gentle reminder that it has been {days} days since the last update on this issue.
Please review the issue and provide any updates or progress in the comments.

Thank you.`;

async function run({ github, context, core, dryRun }) {
  const owner = context.repo.owner;
  const repo = context.repo.repo;
  const now = Date.now();
  const inactivityMs = INACTIVITY_DAYS * MS_PER_DAY;

  let totalChecked = 0;
  let totalReminders = 0;

  core.info(`Starting assignee-reminder (dryRun=${dryRun})`);
  core.info(`Inactivity threshold: ${INACTIVITY_DAYS} days`);

  for await (const { data: issues } of github.paginate.iterator(
    github.rest.issues.listForRepo,
    {   
        owner, 
        repo, 
        state: "open", 
        assignee: "*", 
        per_page: 100,
    },
  )) {
    for (const issue of issues) {
      if (issue.pull_request) continue;

      totalChecked++;

      const reminded = await processIssue({
        github,
        owner,
        repo,
        issue,
        now,
        inactivityMs,
        dryRun,
        core,
      });

      if (reminded) totalReminders++;
    }
  }

  core.info("--- Summary ---");
  core.info(`Total issues checked: ${totalChecked}`);
  core.info(`Total reminders sent: ${totalReminders}`);
}

async function processIssue({
  github,
  owner,
  repo,
  issue,
  now,
  inactivityMs,
  dryRun,
  core,
}) {
  const assignees = issue.assignees.map((a) => a.login);
  const assigneeSet = new Set(assignees.map((a) => a.toLowerCase()));

  const activityByAssignee = {};
  let lastReminderAt = 0;

  for await (const { data: comments } of github.paginate.iterator(
    github.rest.issues.listComments,
    { owner, repo, issue_number: issue.number, per_page: 100 },
  )) {
    for (const comment of comments) {
      if (comment.body?.includes(REMINDER_MARKER)) {
        const t = new Date(comment.created_at).getTime();
        lastReminderAt = Math.max(lastReminderAt, t);
        continue;
      }

      const login = comment.user?.login?.toLowerCase();
      if (login && assigneeSet.has(login)) {
        const t = new Date(comment.created_at).getTime();
        activityByAssignee[login] = Math.max(activityByAssignee[login] || 0, t);
      }
    }
  }

  for await (const { data: events } of github.paginate.iterator(
    github.rest.issues.listEvents,
    { owner, repo, issue_number: issue.number, per_page: 100 },
  )) {
    for (const event of events) {
      if (event.event === "assigned" && event.assignee) {
        const login = event.assignee.login.toLowerCase();
        if (assigneeSet.has(login)) {
          const t = new Date(event.created_at).getTime();
          activityByAssignee[login] = Math.max(activityByAssignee[login] || 0, t);
        }
      }
    }
  }

  const issueCreatedAt = new Date(issue.created_at).getTime();
  const lastAssigneeActivity = Math.max(
    ...Object.values(activityByAssignee),
    issueCreatedAt,
  );

  if (now - lastAssigneeActivity < inactivityMs) return false;

  if (lastReminderAt > lastAssigneeActivity && now - lastReminderAt < inactivityMs) {
    return false;
  }

  const mentions = assignees.map((a) => `@${a}`).join(", ");
  const days = Math.floor((now - lastAssigneeActivity) / MS_PER_DAY);

  if (dryRun) {
    core.info(
      `[DRY RUN] #${issue.number}: Would remind ${mentions} (inactive for ${days} days)`,
    );
    return true;
  }

  const body = COMMENT_TEMPLATE
    .replace("{mentions}", mentions)
    .replace("{days}", days);

  await github.rest.issues.createComment({
    owner,
    repo,
    issue_number: issue.number,
    body,
  });

  core.info(`#${issue.number}: Reminder sent to ${mentions} (inactive for ${days} days)`);
  return true;
}

module.exports = { run };
