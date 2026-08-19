# API documentation rules

Guidelines for documenting functions, types, and other public items. Module-level docs use `##` headings (see [DEVELOPER-GUIDELINES.md](./DEVELOPER-GUIDELINES.md)); **item-level** docs use rustdoc's special `#` sections so they render correctly.

Rustdoc in esp-hal is **reference material**. Follow [ASD-STE100](#simplified-technical-english-asd-ste100) for clarity and the [Espressif Manual of Style (EMoS)](https://mos.espressif.com/) for Espressif-specific conventions. Use second person only in procedural text (`# Examples`, module `## Examples`).

- Start with a one-sentence summary in third-person present tense (e.g. `Sets the baud rate.`, not `Set the baud rate.` or `This function sets the baud rate.`).
- Keep verb tense consistent in a single summary. If the summary starts with a third-person verb (`Starts`, `Checks`, `Returns`, …), conjugate every coordinated verb the same way:
  - Good: `Starts and waits for a conversion on the specified pin and returns the result.`
  - Bad: `Starts and wait … and return …`
- For boolean predicates, use `Returns whether …` and keep a space before the next word:
  - Good: `Returns whether sampling is complete.`
  - Bad: `Returns whethersampling is complete.` / `Returns whetherthe interrupt is set.`
- When a function returns a handle or transaction object, say what it is for:
  - Good: ``Returns a [`TxTransaction`] that can be used to wait for the transaction to complete.``
  - Bad: ``Returns a [`TxTransaction`] to wait for the transaction to complete.`` (reads as if the type itself waits)
- Use hyphens in compound modifiers where needed (`one-time sampling`, not `onetime sampling`).
- Constructors should begin with `Creates a new …` (e.g. ``Creates a new UART instance in [`Blocking`] mode.``).
- Use these section headings where applicable — as `#` headings, not `##`:
  - `# Errors` for fallible functions
  - `# Panics` for functions that may panic
  - `# Safety` for `unsafe` functions
  - `# Cancellation Safety` for async functions and futures
  - `# Examples` for doctests (prefer `rust, no_run` and `before_snippet!()` / `after_snippet!()`) - prefer `# Examples` over `# Usage`
- Avoid empty sections. If a section has no body (for example `# Examples` with no code fence, or `# Errors` with no conditions), remove the heading entirely — do not leave a bare `# Examples` / `# Errors` / `# Panics` / `# Safety` line above the item.
- The order of sections should be `# Examples`, `# Panics`, `# Errors`, `# Cancellation Safety`, `# Safety`, `# Undefined Behavior`
- Do not repeat the summary in section bodies (e.g. put error conditions under `# Errors`, not in a leading `Returns a … if …` paragraph before the section).
- Prefer rustdoc intra-doc links (e.g. ``[`Type`]``) over raw URLs or markdown links to rustdoc items.
  - Format API names as code: ``[`DelayNs`]``, not `[DelayNs]`.
  - Leave a space before an intra-doc link when it follows a preposition or ordinary word:
    - Good: ``Used with [`Self::register_block`] to access the register block.``
    - Bad: `Used with[Self::register_block] to access the register block.`
  - After bulk edits, check that words were not merged across boundaries (`whether` + `the`, `with` + ``[`Type`]``, `that` + `can`, etc.).
  - Don't merge regular comments and cfg-gated (cfg_attr) comments.
- Do not leave stray or accidental characters in doc comments (keyboard typos, dead keys, paste artifacts). Summaries must end with normal ASCII punctuation (`.`, `?`, or `!`) — not letters stuck after the full stop (e.g. `Sets the output level.ç`).

## Mechanical doc edits

Avoid repository-wide search-and-replace on doc comments (e.g. `Check if` → `Returns whether`, imperative → third person) without reviewing each hunk.

Style/tense passes alone are not enough. Also scan for empty sections, copy-paste mistakes, and stray characters — those do not show up in the usual “imperative → third person” greps.

After any bulk doc pass:

1. Read the `git diff` for doc-only changes, not only whether CI is green.
2. Grep for common regressions before opening the PR:
   - merged words: `whetherthe`, `whethersampling`, `with[`, `than can`
   - mixed tense: `(Starts|Checks|Returns).+ and (wait|write|return|copy)\b`
   - typos: `Fow `, `Finishes of`, British spellings (`behaviour`, `whilst`, `initialised`)
   - empty sections: `# Examples` / `# Errors` / `# Panics` / `# Safety` / `# Cancellation Safety` immediately followed by the next item or another `#` heading with no body
   - stray characters: unexpected non-ASCII in `///` / `//!` lines (e.g. `ç`, `Ã`, mojibake), or a letter immediately after a sentence-ending `.` (`level.ç`, `pin.x`)
3. Run `cargo xtask build documentation --packages=esp-hal --chips=<chip>` (esp-hal denies `rustdoc::all` warnings).

## Simplified Technical English (ASD-STE100)

API documentation should follow [ASD-STE100](https://www.asd-ste100.org/) where practical. STE is a controlled-language standard: it limits ambiguous wording so technical text has one clear meaning. That helps readers who use English as a second language and keeps docs easier to parse. Pair STE with [EMoS](#espressif-manual-of-style-emos) for Espressif product naming and reference-style voice.

We apply STE **principles** to rustdoc, not a word-by-word check against ASD’s full approved dictionary (~900 words). For exact ASD wording, use the [official standard](https://www.asd-ste100.org/). Do not use STE for marketing or narrative copy where tone matters.

### Style

- Use short sentences. Put one idea in each sentence. Aim for about 20 words or fewer in summaries and procedure-style lines; up to about 25 words in longer descriptions before splitting.
- Use the present tense and active voice.
- Prefer simple tenses. Use simple present for behavior (e.g. “Returns the baud rate.”). Avoid present perfect (“has returned”) unless the timing relative to another event matters.
- Passive voice is acceptable when the actor is irrelevant and the doc should emphasize the peripheral, register, or result (e.g. “The reset source code is stored in `LP_CLKRST_RESET_CAUSE`.”).
- Do not start item summaries with “This function”, “This struct”, “This enum”, or “This error”.
- Do not use “Note that”. State the fact directly. Prefer `# Panics`, `# Safety`, and `# Errors` over informal “Note:” callouts in item docs.
- Use “must” for requirements, “can” for ability, and “do not” for prohibitions.
- Avoid “will” when the present tense is enough.
- Avoid “allows you to”. Use “lets” or rewrite the sentence.
- Keep subjects and verbs explicit. Do not drop words to save space if that makes the meaning unclear.
- Do not use contractions (`can't`, `don't`, `it's`). Write the full words.

### Wording

- **One word, one meaning.** Pick one verb for one action and use it consistently (e.g. always “Returns” for fallible results, not “Returns” in one place and “Gives back” in another).
- Do not replace a longer but correct phrase with a shorter ambiguous one. Keep `that can be used to` when the returned value is a tool for a later action.
- **One part of speech per word.** If a word is a noun in one doc, do not use it as a verb elsewhere unless that is normal English for the domain.
- **Short noun phrases.** Keep stacked nouns to about three words (e.g. “DMA transfer buffer”). Split longer phrases into a short clause.
- **Domain terms.** Keep necessary peripheral and chip names (`TWAI`, `eFuse`, …). Define uncommon terms once in module docs or link to the TRM / a glossary. Do not replace a precise technical term with a vague synonym. Use Espressif-preferred spellings from [EMoS](#espressif-manual-of-style-emos).

### Structure

- In `# Errors`, name the error type and the condition (e.g. ``[`ConfigError`] when the baud rate is not supported.``). Do not write “Returns a … if …”.
- Predicate helpers belong in the summary, not under `# Errors` (e.g. `Returns whether the timer has been configured.`, not “Returns a bool if …”).
- Enum variant docs are noun phrases or short statements, not “This error occurs when …”.
- Field docs use the same third-person style as function summaries where they are full sentences.
- Use `# Examples` or bullet lists for three or more steps, conditions, or options. Do not bury a sequence in one long sentence.
- Keep one topic per paragraph in longer module docs (roughly six sentences or fewer).
- In lists of complete sentences, end each item with a full stop. In lists of fragments, omit end punctuation.

### Limits

- Do not shorten a sentence if that drops a safety condition, exception, scope qualifier, or numeric limit. Keep the longer phrasing when precision matters.
- STE favors plain, literal prose. That is appropriate for API docs; it is not a goal for changelog voice or user-facing tutorials outside the HAL reference.

## Espressif Manual of Style (EMoS)

[EMoS](https://mos.espressif.com/) is Espressif’s documentation style guide. The [quick reference](https://mos.espressif.com/quick-reference-guide/quick-reference-guide__EN.html) covers most rules relevant to esp-hal rustdoc. EMoS targets manuals and guides as well as reference; apply only what fits API docs.

### Point of view

- **Reference (default):** Item summaries, `# Errors`, field docs, and module overview text use an **impersonal** voice — describe what the type or peripheral does, not who uses it.
- **Procedures:** `# Examples` and module `## Examples` may use **second person** and the imperative mood (`Configure the pins.`, `Create a [`Uart`]`).
- Do not address the reader as “users”, “the user”, or “you” outside examples.
- Do not use “we” or “I” in rustdoc.
- Do not switch point of view within one doc comment or module section.

### Spelling and terms

- Use **American English** spelling (e.g. `center`, `color`, `initialize`).
- Common British forms to avoid: `behaviour`, `whilst`, `initialised`/`initialise`, `signalled`, `undefined behaviour` — use the American equivalents.
- Spell out an abbreviation or acronym on **first use** in a module (e.g. `two-wire automotive interface (TWAI)`). Skip the expansion when the short form is more familiar than the full term (`USB`, `DMA`).
- Use technical terms **consistently** across drivers. When unsure, check Espressif’s published docs and TRM naming before inventing a synonym.
- Prefer established Espressif forms: `eFuse` (not `Efuse`), `Wi-Fi` (hyphenated), register and field names as in the TRM (`EFUSE_CONF_REG`, `EFUSE_OP_CODE`).

### Numbers, units, and hardware values

- In prose, spell out **one through nine**; use numerals for **10 and higher**, unless all numbers in the same category should match (e.g. a list of interface counts).
- Use **numerals** for measurements, bit widths, clock rates, and parameters (`8-bit`, `40 MHz`, `115200` baud).
- Put a **space** between a number and its unit (`40 MHz`, `85 °C`, `10 ms`).
- For addresses, register values, and bit patterns, use **hexadecimal** with an `0x` prefix (`0x9000`, `0x5A5A`). Do not use Verilog-style literals (`1'b1`, `1'h0`).
- Use an **en dash** without spaces for numeric ranges (`GPIO0–GPIO3`, `0x1000–0x2000`).

### Modes and headings

- For hardware or driver **mode** names, use title case for the name and lowercase `mode` (`Station mode`, `Light-sleep mode`). Do not put `the` before a mode name.
- Module `##` headings use title case (`## Usage`, `## Implementation State`). Rustdoc `#` sections follow Rust convention (`# Errors`, `# Examples`).

### Punctuation

- Use the **Oxford comma** in lists of three or more items.
- Do not use `/` to mean “or” (`UART or SPI`, not `UART/SPI`).
- Use English punctuation in English docs.
- Prefer ASCII punctuation in item summaries (`.`, `,`, `;`, `:`). Intentional Unicode (en dashes in ranges, `µs`, curly quotes in quoted TRM text) is fine; accidental accents or paste garbage is not.
