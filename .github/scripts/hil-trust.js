// .github/scripts/hil-trust.js

const TRUST_HEADER = "### [HIL trust list]";
const JSON_BEGIN = "<!-- HIL_TRUST_JSON";
const JSON_END = "HIL_TRUST_JSON -->";

function parseTrustCommand(body, cmd) {
  const text = String(body || "").trim();
  const re = new RegExp(`^\\/${cmd}\\s+@([A-Za-z0-9-]+)\\b$`, "i");
  const m = text.match(re);
  if (!m) return { login: "", error: `Usage: /${cmd} @<login>` };
  return { login: m[1].toLowerCase(), error: "" };
}

function extractTrustedList(existingBody) {
  const existing = String(existingBody || "");
  const m = existing.match(/<!--\s*HIL_TRUST_JSON\s*([\s\S]*?)\s*HIL_TRUST_JSON\s*-->/);
  if (!m || !m[1]) return [];
  try {
    const data = JSON.parse(m[1]);
    return Array.isArray(data.trusted) ? data.trusted.map(s => String(s).toLowerCase()) : [];
  } catch {
    return [];
  }
}

function renderTrustBody(list) {
  const trusted = Array.from(new Set(list.map(s => String(s).toLowerCase()))).sort();
  const json = JSON.stringify({ trusted }, null, 2);
  const pretty = trusted.length ? trusted.map(u => `- @${u}`).join("\n") : "_None yet_";

  const body =
`${TRUST_HEADER}
${JSON_BEGIN}
${json}
${JSON_END}

<details>
<summary>Trusted users for this PR (click to expand)</summary>

${pretty}

</details>`;

  return { body, trusted };
}

function upsertTrusted(existingBody, login) {
  const list = extractTrustedList(existingBody);
  if (login && !list.includes(login)) list.push(login);
  return renderTrustBody(list);
}

function revokeTrusted(existingBody, login) {
  const list = extractTrustedList(existingBody).filter(u => u !== String(login || "").toLowerCase());
  return renderTrustBody(list);
}

module.exports = {
  parseTrustCommand,
  upsertTrusted,
  revokeTrusted,
};