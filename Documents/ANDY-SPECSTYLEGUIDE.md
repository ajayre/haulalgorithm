# Andy specification style guide **[0.0.1]**

**[0.0.1]** **Document type:** Authoring guide for normative technical specifications

**[0.0.2]** This guide defines how Andy writes and maintains specifications: structure, traceability, normative language, cross-document relationships, wire/API layout, and supporting artifacts (registries, addenda, checklists). New or revised specs **shall** follow these rules unless a specific document explicitly documents a one-off exception.

---

## 1. Document family and file naming

### 1.1 Primary specification

**[1.1.1]** The **baseline** document is a single large markdown file named **`PROJECT-SPEC.md`** (uppercase, hyphen-separated words, `.md` extension). Example: **`CDNET-SPEC.md`**.

**[1.1.2]** The title line uses **`#`** followed by the human-readable product name and the words **Specification**, with the **document paragraph-id suffix** on the same line:

```markdown
# CDNet Specification **[0.0.1]**
```

**[1.1.3]** The baseline spec is the **authoritative** contract for the core library or system. It is written to be **implementation-complete** for the reference product: architecture, wire format, public API, build layout, logging, testing expectations, and phased delivery (**Step 1** / **Step 2** or equivalent).

### 1.2 Application- or protocol-layer specification

**[1.2.1]** A protocol that runs **on top of** the baseline (without changing the baseline wire contract) gets its own file, typically **`SERVICE_LAYER.md`** or **`PROTOCOL-NAME-SPEC.md`**.

**[1.2.2]** The opening **§0** block **shall** state **Relationship to** the baseline spec, what the layer **does not** change, and where the native implementation lives (separate module names and deliverables).

### 1.3 Addenda

**[1.3.1]** **Backward-compatible extensions** that would clutter the frozen baseline are published as **`PROJECT-SPEC-ADDENDAN.md`** (sequential addendum number). Example: **`CDNET-SPEC-ADDENDA1.md`**.

**[1.3.2]** Each addendum **shall** include:

- **Baseline specification** filename and scope of the addendum.
- **Relationship to baseline** — baseline remains normative unless the addendum **explicitly supersedes** a paragraph.
- **Section-number alignment** — addendum sections **extend** the same section numbers as the baseline (e.g. addendum **§15.3** extends baseline **§15.3**).
- Its own **§0.1** revision history table.

**[1.3.3]** Addenda **shall not** silently redefine unrelated baseline behavior. Client-visible changes that do not need new API may still be specified (e.g. monotonic ID assignment in **§10.0.1** of an addendum).

### 1.4 Shared registries

**[1.4.1]** Numeric identifiers shared across protocols (**`Protocol`**, **`Channel`**, message types, status codes) live in a dedicated **`PROJECT-PROTOCOL-REGISTRY.md`** (or similar), not duplicated as the sole authority in multiple specs.

**[1.4.2]** The registry is the **single authoritative assignment** for those numbers. Protocol specs **shall** reference registry sections and **shall** require new IDs to be **claimed in the registry first**.

**[1.4.3]** Registry documents are shorter, table-heavy, and include **rules for claiming** new values, **range layout** in ` ```text ` blocks, **normative `#define` fragments** in ` ```c ` blocks, and a **revision history** (often unnumbered **§7** or **§0.1**).

### 1.5 What not to put in a spec

**[1.5.1]** Implementation source code belongs in the tree described by **Build** / **Reference implementation** sections, not inlined as the primary artifact. Specs **may** include **normative header sketches** (**§30**-style) and **fragments** sufficient to implement wire layouts and public API.

**[1.5.2]** Personal notes, session scratchpads, and agent prompts belong outside normative specs (e.g. **`NOTES.md`**), not mixed into **`§0`**–**§N`** normative sections.

---

### 1.6 File encoding (UTF-8, not UTF-16)

**[1.6.1]** Specification markdown files **shall** be stored as **UTF-8 without BOM**.

**[1.6.2]** **Problem.** On OneDrive-backed paths, some editor and agent tools have been observed to save markdown as **UTF-16 LE**. That breaks **`grep`**, **`Read`/`StrReplace`** in agent sessions, **`number_spec_paragraphs.py`** (which opens specs as UTF-8), and automated patch scripts. A UTF-16 file typically starts with bytes **`23 00 20 00`** (`#` + space in UTF-16) instead of **`23 20`** (`#` + space in UTF-8).

**[1.6.3]** **Verification.** After bulk edits, confirm encoding before sign-off:

```text
UTF-8 (good):    first bytes  23 20 45 6D 53 41   (# EmSA...)
UTF-16 LE (bad): first bytes  23 00 20 00 45 00   (# with null bytes between ASCII)
```

**[1.6.4]** **Safe write pattern (PowerShell).** When patching specs programmatically, use **`UTF8Encoding(false)`** (no BOM):

```powershell
$utf8 = New-Object System.Text.UTF8Encoding $false
[IO.File]::WriteAllText($path, $text, $utf8)
# Alternative: [IO.File]::WriteAllBytes($path, [Text.Encoding]::UTF8.GetBytes($text))
```

**[1.6.5]** **Safe write pattern (Python).** Use **`Path.write_bytes(text.encode("utf-8"))`**. Prefer piping patch logic on **stdin** (`py -3 -`) when a helper **`*.py`** file might be saved as UTF-16 by the same tooling (Python then fails with *source code string cannot contain null bytes*).

**[1.6.6]** **After external patches.** If the spec looks correct on disk but wrong in the IDE, **reload from disk** — the editor may still hold a stale UTF-16 buffer that overwrites a good UTF-8 save on the next auto-save.

**[1.6.7]** **Normative tools.** **`number_spec_paragraphs.py`**, **`git diff`**, and CI checks **shall** assume UTF-8. Do **not** commit UTF-16 spec files.

---

## 2. Paragraph identifiers (traceability)

### 2.1 Format

**[2.1.1]** Every normative paragraph carries a bold marker **`[<S>.<U>.<P>]`**:

| Component | Meaning |
|---|---|
| **`<S>`** | Section number from the nearest preceding **`## <S>.`** heading |
| **`<U>`** | Subsection from **`### <S>.<U>`** when present; **`0`** for body text before the first **`###`** in that section |
| **`<P>`** | 1-based paragraph index within that **`<S>.<U>`** group |

**[2.1.2]** Example: **`[15.14.3]`** is the third paragraph in **§15.14** (or in subsection **15.14** if the heading is **`### 15.14`**).

### 2.2 Placement rules

**[2.2.1]** **Prefix** — On ordinary prose lines and list items, the marker is the **first** content on the line:

```markdown
**[1.0.1]** CDNet is a platform-independent C library that creates a private, local-only message network.
```

**[2.2.2]** **Suffix** — On the **document title** (`# ...`) and on **unnumbered** **`###`** headings (e.g. **`### 0.1 Specification revision history`**), the marker is a **suffix**:

```markdown
# CDNet Specification **[0.0.1]**
### 0.1 Specification revision history
```

**[2.2.3]** **Code fences** — The marker sits on **its own line**, then a **blank line**, then the opening fence at **column 0** (the script enforces this; see **§2.5**):

**[2.2.4]** **Never** place a paragraph id on the same line as a **` ``` `** opening or closing fence. Inline code with **single** backticks on the id line (e.g. **`**[10.0.2]** `cdnet_client_id_t` is …`**) is allowed for prose.

### 2.3 Section 0 convention

**[2.3.1]** **§0** (section **`0`**, subsection **`0`**) is reserved for **document metadata** not tied to a numbered **`## 1.`** chapter: paragraph-id legend, project name, document type, scope, related specs, version axes, shipping artifacts, project phases.

**[2.3.2]** Paragraphs **`[0.0.1]`** through **`[0.0.N]`** form the metadata block before the first **`---`** horizontal rule and **`## 1.`** chapter.

### 2.4 Maintenance

**[2.4.1]** Regenerate paragraph numbers after insertions, deletions, or moves. The runnable copy lives at **`tools/number_spec_paragraphs.py`**; the **canonical verbatim source** for extraction is **§2.5** below (keep both in sync when editing the tool).

**[2.4.2]** When citing obligations in checklists, tests, or PRs, prefer **paragraph ids** (e.g. **`§15.4.20`**, **`[15.4.20]`**) over vague section titles.

**[2.4.3]** Default files processed when run with no arguments: **`CDNET-SPEC.md`**, **`CDNET-SPEC-ADDENDA1.md`**, **`SERVICE_LAYER.md`**, **`CDNET-PROTOCOL-REGISTRY.md`**. Pass additional paths on the command line to number other markdown files (e.g. **`ANDY-SPECSTYLE.md`**).

### 2.5 Verbatim tool — `tools/number_spec_paragraphs.py`

**[2.5.1]** **Extraction boundaries (for agentic tools).** Copy the Python source **exactly** from the line **`# BEGIN number_spec_paragraphs.py`** through **`# END number_spec_paragraphs.py`** inclusive, then write it to **`tools/number_spec_paragraphs.py`** (create **`tools/`** if missing). Run from the CDNet SDK root:

```text
python tools/number_spec_paragraphs.py
python tools/number_spec_paragraphs.py ANDY-SPECSTYLE.md
```

**[2.5.2]** The tool **shall not** leave a paragraph id and a **` ``` `** fence on the same line; it splits legacy combined lines, emits ids above fences, and **raises** if the output still violates that rule.

**[2.5.3]**

<!-- AI: extract from # BEGIN through # END below into tools/number_spec_paragraphs.py -->

````python
# BEGIN number_spec_paragraphs.py
"""
Number markdown paragraphs in CDNet spec documents as **\[S.U.P]** for traceability.

- Section S from "## S. Title".
- Subsection U from "### S.U ..." matching current S; preamble / intro uses U=0.
- Paragraph P increments within each (S, U) block.
- Unnumbered "### Heading" does not change U or reset P; the heading line gets the next P
  as a suffix so the line still starts with ### (valid Markdown).
- Top-level "# Title" uses a suffix ID for the same reason.
- Code fences: one ID emitted as a paragraph on its **own line** above the
  opening fence (with a blank line separating it from the fence), so the
  fence stays at column 0 and Markdown renderers can syntax-highlight the
  fenced code. Inline "**[S.U.P]** ```" lines are split before re-numbering.
- List lines (-, *, N.) each get their own ID prefix.
- Prose runs until a blank line (soft-wrapped lines share one ID on the first line).
- Strips existing paragraph IDs (line-start prefix, line-end suffix, or standalone
  marker above a fence) so re-runs are safe.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_SPEC_FILES = (
    ROOT / "CDNET-SPEC.md",
    ROOT / "CDNET-SPEC-ADDENDA1.md",
    ROOT / "SERVICE_LAYER.md",
    ROOT / "CDNET-PROTOCOL-REGISTRY.md",
)

PID_INNER = r"\d+\.\d+\.\d+(?:[a-z])?"
LINE_PID_PREFIX_RE = re.compile(rf"^\*\*\[{PID_INNER}\]\*\* ")
LINE_PID_SUFFIX_RE = re.compile(rf"\s+\*\*\[{PID_INNER}\]\*\*\s*$")
LINE_PID_STANDALONE_RE = re.compile(rf"^\*\*\[{PID_INNER}\]\*\*\s*$")
# Paragraph id and opening fence must never share a line (breaks Markdown highlighting).
INLINE_FENCE_RE = re.compile(rf"^(\*\*\[{PID_INNER}\]\*\*)\s+(```.*)$")
PID_LINE_CONTAINS_FENCE_RE = re.compile(rf"^\*\*\[{PID_INNER}\]\*\* .*```")


def is_h2(line: str) -> bool:
    return line.startswith("## ") and not line.startswith("###")


def is_h3(line: str) -> bool:
    return line.startswith("### ") and not line.startswith("####")


def parse_h2(line: str) -> int | None:
    m = re.match(r"^## (\d+)\.", line)
    return int(m.group(1)) if m else None


def parse_h3_numeric(line: str, current_section: int) -> tuple[int, bool]:
    """Returns (subsection U, True) if line is ### S.U ... for current S; else (_, False)."""
    m = re.match(r"^### (\d+)\.(\d+)(\s|$)", line.rstrip("\r\n"))
    if m:
        s, u = int(m.group(1)), int(m.group(2))
        if s == current_section:
            return u, True
    return 0, False


def is_list_item(line: str) -> bool:
    s = line.lstrip()
    if not s:
        return False
    if re.match(r"^[-*] \S", s):
        return True
    if re.match(r"^\d+\. \S", s):
        return True
    return False


def split_trailing_nl(line: str) -> tuple[str, str]:
    if line.endswith("\r\n"):
        return line[:-2], "\r\n"
    if line.endswith("\n"):
        return line[:-1], "\n"
    return line, ""


def strip_line_ids(line: str) -> str:
    body, nl = split_trailing_nl(line)
    body = LINE_PID_PREFIX_RE.sub("", body, count=1)
    body = LINE_PID_SUFFIX_RE.sub("", body)
    return body + nl


def expand_inline_fence_markers(lines: list[str]) -> list[str]:
    """Split `**[S.U.P]**` and an opening ``` fence that share one line."""
    out: list[str] = []
    for line in lines:
        body, nl = split_trailing_nl(line)
        m = INLINE_FENCE_RE.match(body)
        if m:
            out.append(m.group(1) + nl)
            out.append(nl)
            out.append(m.group(2) + nl)
        else:
            out.append(line)
    return out


def assert_no_pid_on_fence_line(md: str, path: Path | None = None) -> None:
    """Raise if any line puts a paragraph id and a ``` fence on the same line."""
    label = f"{path}: " if path else ""
    for lineno, line in enumerate(md.splitlines(), start=1):
        body, _ = split_trailing_nl(line)
        if PID_LINE_CONTAINS_FENCE_RE.match(body):
            raise ValueError(
                f"{label}line {lineno}: paragraph id and ``` fence must be on "
                f"separate lines (blank line between marker and fence)"
            )


def strip_all_ids(md: str) -> str:
    lines = expand_inline_fence_markers(md.splitlines(keepends=True))
    out: list[str] = []
    i = 0
    while i < len(lines):
        line = lines[i]
        body, _ = split_trailing_nl(line)
        if LINE_PID_STANDALONE_RE.match(body):
            # A whole-line marker (used above a code fence). Drop the marker
            # line *and* the following blank line we inserted with it, so the
            # blank-line accounting stays stable across re-runs.
            i += 1
            if i < len(lines) and not lines[i].strip():
                i += 1
            continue
        out.append(strip_line_ids(line))
        i += 1
    return "".join(out)


def next_pid(section: int, subsection: int, para_holder: list[int]) -> str:
    para_holder[0] += 1
    return f"**[{section}.{subsection}.{para_holder[0]}]**"


def number_spec(md: str) -> str:
    md = strip_all_ids(md)
    lines = md.splitlines(keepends=True)
    out: list[str] = []

    section = 0
    subsection = 0
    para = [0]

    i = 0
    n = len(lines)

    def reset_para() -> None:
        para[0] = 0

    while i < n:
        line = lines[i]

        if is_h2(line):
            p = parse_h2(line)
            if p is not None:
                section = p
                subsection = 0
                reset_para()
            out.append(line)
            i += 1
            continue

        if is_h3(line):
            u, matched = parse_h3_numeric(line, section)
            if matched:
                subsection = u
                reset_para()
                out.append(line)
            else:
                pid = next_pid(section, subsection, para)
                body, nl = split_trailing_nl(line)
                out.append(f"{body} {pid}{nl}")
            i += 1
            continue

        stripped = line.strip()
        if stripped == "---":
            out.append(line)
            i += 1
            continue

        if not stripped:
            out.append(line)
            i += 1
            continue

        if stripped.startswith("#") and not stripped.startswith("##"):
            pid = next_pid(section, subsection, para)
            body, nl = split_trailing_nl(line)
            out.append(f"{body} {pid}{nl}")
            i += 1
            continue

        if line.lstrip().startswith("```"):
            pid = next_pid(section, subsection, para)
            # Emit the marker as its own paragraph, with a blank line between
            # it and the fence, so the fence stays at column 0 and Markdown
            # renderers (including Cursor's preview) still recognise it.
            if out and out[-1].strip():
                out.append("\n")
            out.append(pid + "\n")
            out.append("\n")
            out.append(line)
            i += 1
            while i < n:
                cur = lines[i]
                out.append(cur)
                if cur.strip() == "```":
                    i += 1
                    break
                i += 1
            continue

        if is_list_item(line):
            pid = next_pid(section, subsection, para)
            out.append(pid + " " + line)
            i += 1
            continue

        body, _ = split_trailing_nl(line)
        if body.lstrip().startswith("```"):
            # Defensive: treat as fence even if leading whitespace slipped through.
            pid = next_pid(section, subsection, para)
            if out and out[-1].strip():
                out.append("\n")
            out.append(pid + "\n")
            out.append("\n")
            out.append(line)
            i += 1
            while i < n:
                cur = lines[i]
                out.append(cur)
                if cur.strip() == "```":
                    i += 1
                    break
                i += 1
            continue

        pid = next_pid(section, subsection, para)
        out.append(pid + " " + line)
        i += 1
        while i < n:
            nxt = lines[i]
            if not nxt.strip():
                out.append(nxt)
                i += 1
                break
            if is_h2(nxt) or is_h3(nxt):
                break
            if nxt.strip().startswith("#") and not nxt.strip().startswith("##"):
                break
            if nxt.lstrip().startswith("```"):
                break
            if is_list_item(nxt):
                break
            out.append(nxt)
            i += 1

    result = "".join(expand_inline_fence_markers(out))
    assert_no_pid_on_fence_line(result)
    return result


def main() -> None:
    paths = [Path(p) for p in sys.argv[1:]] if len(sys.argv) > 1 else list(DEFAULT_SPEC_FILES)
    for path in paths:
        text = path.read_text(encoding="utf-8")
        numbered = number_spec(text)
        assert_no_pid_on_fence_line(numbered, path)
        path.write_text(numbered, encoding="utf-8", newline="\n")
        print(f"Numbered paragraphs in {path}")


if __name__ == "__main__":
    main()
# END number_spec_paragraphs.py
````


## 3. Heading hierarchy and section numbering

### 3.1 Levels

**[3.1.1]**

| Level | Markdown | Numbering | Typical content |
|---|---|---|---|
| Document title | `#` | Suffix **`[0.0.1]`** on title only | Product name + “Specification” |
| Chapter | `##` | **`## <N>. Title`** | Purpose, Design Goals, Wire Format, Public API, … |
| Section | `###` | **`### <N>.<M> Title`** or **`### <N>.<M> Title (normative)`** | Subsystems, API subsections, message families |
| Subsection | `####` | Rare; used in coding style guides, not in the four CDNet specs | Avoid unless depth truly requires it |

**[3.1.2]** Top-level chapters use **integers only** (`## 1.`, `## 12.`, `## 34.`). **Gaps** in numbering are allowed when a section is reserved or removed (e.g. no **`## 33.`**).

**[3.1.3]** **`###`** headings under a chapter **repeat the chapter number**: **`### 12.6 Source Client ID`**, not **`### Source Client ID`** alone.

### 3.2 Standard chapter order (baseline library spec)

**[3.2.1]** A full library specification **should** follow this spine (adapt names, omit irrelevant chapters):

1. **Purpose**
2. **Design Goals** (numbered **`shall provide`** list)
3. **Non-Goals** (numbered list; optional “not non-goals” clarifications)
4. **Terminology** (**`### <N>.<M>`** per defined term)
5. **High-Level Architecture** (diagrams + layer stack)
6. **Platform / transport / integration** (as needed)
7. **Lifecycle, IDs, routing** (domain-specific)
8. **Wire format / envelope**
9. **Registries** (or pointer to external registry file)
10. **Public API** (largest section; **`###`** per topic)
11. **Thread safety, performance, security**
12. **Logging and diagnostics**
13. **Build requirements / deliverables**
14. **Normative header sketch** (optional **`## 30.`**)
15. **Open design decisions** (historical **`(Decided)`** record)
16. **MVP / phased feature set**
17. **Summary**

**[3.2.2]** Application-layer specs **should** add: **CDNet integration**, **message catalog**, **server/client behavior**, **native API header**, **hardware adapter**, **automated testing**, **managed wrapper**, **implementation checklist**.

### 3.3 Horizontal rules

**[3.3.1]** Use **`---`** on its own line **between major chapters** (after **§0** metadata and between **`## N.`** sections) for visual separation in long documents. Not required between every **`###`**.

### 3.4 Normative labeling in headings

**[3.4.1]** When a subsection is entirely binding, append **`(normative)`** to the **`###`** title:

```markdown
### 5.1 Implementation stack (normative)
### Time bounds (normative) **[15.0.5]**
```

**[3.4.2]** Informative sections **shall** say so in the opening paragraph (e.g. **“This section illustrates … These are informative, not normative.”**).

---

## 4. Section 0 metadata block

### 4.1 Required fields

**[4.1.1]** After **`[0.0.2]`** (paragraph-id rules), include a labeled metadata list. Typical keys:

- **Project name**
- **Document type** (e.g. Initial library specification, Application-level protocol specification, Normative addendum, Shared registry)
- **Language / implementation** (C, C#, wire-only, …)
- **Target platforms**
- **Scope** (one sentence boundary)

**[4.1.2]** **Service-layer** and **addendum** specs **shall** add **Relationship to** baseline and **Assumed knowledge** where readers must already know the base spec.

### 4.2 Version axes (do not conflate)

**[4.2.1]** When multiple “versions” exist, dedicate **`[0.0.6]`** or similar to **Version axes** with a **numbered list**:

1. **Specification document** version (this markdown file; semver in **§0.0.3** and **§0.1** table)
2. **Wire protocol** version constants (`*_VERSION_MAJOR` / `MINOR` in headers)
3. **Library build** version (`*_get_library_version()`)

**[4.2.2]** State explicitly that incrementing one axis **does not** imply incrementing another.

### 4.3 Revision history

**[4.3.1]** Use **`### 0.1 Specification revision history`** (unnumbered **`###`**, suffix id on subsection title optional) with a markdown table:

| Version | Date | Change |
|---|---|---|
| **1.0.0** | YYYY-MM-DD | Initial … |

**[4.3.2]** Follow with a paragraph requiring **[semantic versioning](https://semver.org/)** rules: **MAJOR** = incompatible normative change, **MINOR** = backward-compatible addition, **PATCH** = clarification without obligation change.

**[4.3.3]** Registries may use a simpler **§7 Revision history** with **Date | Change** only.

### 4.4 Project phases and deliverables

**[4.4.1]** Normative **shipping artifacts** (paths, binary names, VS projects) belong in **§0** and/or **§29**-style build sections, with **shall** language.

**[4.4.2]** **Phased delivery** (**Step 1**, **Step 2**) **shall** list what **shall not** ship in earlier steps (e.g. no CAN-FD helpers in Step 1).

---

## 5. Typography and emphasis

### 5.1 Bold

**[5.1.1]** Use **`**bold**`** for:

- Paragraph ids **`[S.U.P]`**
- RFC-style keywords: **shall**, **shall not**, **must**, **must not**, **should**, **may**
- Defined term **first use** in a terminology entry
- Section references: **§12.12**, **§15.4**
- Document references: **CDNET-SPEC.md**, **SERVICE_LAYER.md**
- UI or role names: **Server**, **Client**, **Service Server**
- Normative labels: **(normative)**, **(Decided)**, **Step 1**

### 5.2 Inline code

**[5.2.1]** Use **backticks** for:

- C identifiers: **`cdnet_client_connect()`**, **`cdnet_result_t`**, **`CDNET_OK`**
- Wire fields: **`Protocol`**, **`NetworkId`**, **`MessageType`**
- File and path literals: **`Library/cdnet.dll`**, **`cdservice.h`**
- Numeric literals: **`0x01`**, **`68`** octets
- Instance names: **`"SL.Service"`**

**[5.2.2]** Prefer **`#define`** / **`typedef`** names from normative headers over prose paraphrases.

### 5.3 Italic

**[5.3.1]** Italic is used sparingly in the CDNet specs; prefer bold or code for consistency. Exception: external link text in revision-history semver paragraph.

---

## 6. Normative language

### 6.1 Keyword rules

**[6.1.1]** **shall** / **shall not** — absolute requirement for conforming implementations.

**[6.1.2]** **must** / **must not** — hard requirement, often for safety or cross-protocol coexistence (registry uses **must** for coexistence on shared networks).

**[6.1.3]** **should** — recommendation; deviation needs justification (MVP lists, forward-compatibility notes).

**[6.1.4]** **may** — optional capability.

**[6.1.5]** **Do not** — use in **“Do Not Implement”** checklists; equivalent to **shall not** for the reference product.

### 6.2 Stating obligations

**[6.2.1]** One obligation per paragraph where possible. Numbered sub-rules use **`1.`**, **`2.`** under a lead-in paragraph:

```markdown
**[15.3.3]** **Rules — logical network removal (server only)**

**[15.3.4]** 1. **`cdnet_server_unregister_network()`** shall be called **only** from the server process ...
**[15.3.5]** 2. The call shall fail with **`CDNET_ERROR_INVALID_STATE`** if ...
```

**[6.2.2]** **Design Goals** and **Non-Goals** use **numbered lists** (`1.` … `36.`) at chapter level; **Purpose** and feature lists often use **`-`** bullets.

### 6.3 Decided design issues

**[6.3.1]** Resolved open questions move to **§31 Open Design Decisions** (or equivalent) as **`(Decided)`** numbered items, each pointing to normative sections. Reopened decisions require a spec revision, not silent edits.

---

## 7. Cross-references

### 7.1 Within the same document

**[7.1.1]** Use **§** followed by section number: **§15.4**, **§12.12**. For subsection precision: **§15.4.20**, **§11.8.14**.

**[7.1.2]** Multiple sections: **§0.0.4**–**§0.0.8**, **§15.3.9**–**§15.3.13**.

### 7.2 Other documents

**[7.2.1]**

| Pattern | When |
|---|---|
| **CDNET-SPEC.md §N** or **CDNet §N** | Citing the base library spec from a layer spec |
| **Addendum 1 §N** or **CDNET-SPEC-ADDENDA1.md §N** | Citing an addendum |
| **SERVICE_LAYER.md §N** | Full filename when ambiguous |
| **CDNET-PROTOCOL-REGISTRY.md §2.1** | Registry as authority for numeric IDs |

**[7.2.2]** In the service-layer spec, bare **§N** means **this document**; **CDNet §N** means the base spec. State that rule in **Assumed knowledge** (**§0.0.5**).

### 7.3 Paragraph-level citations

**[7.3.1]** Checklists and implementation guides **shall** cite **§** and paragraph ids together: **§13.2.15**, **Addendum 1 §15.3**, **`[25.9.1]`**.

---

## 8. Lists and enumerations

### 8.1 Design goals

**[8.1.1]** Chapter **§2 Design Goals** opens with **“shall provide:”** then **numbered** items **`1.`** through **`N.`**, each a single capability with parenthetical section pointers (**§8.1**, **§15.4**).

### 8.2 Non-goals

**[8.2.1]** Mirror design goals with **numbered shall not attempt** items. Optional follow-on **“Not non-goals”** bullet list for deliberate future-proofing.

### 8.3 Terminology

**[8.3.1]** Each term is **`### N.M Term name`** followed by one or more **`[N.M.P]`** paragraphs. Optional **Examples:** bullet sublist.

### 8.4 Feature / deliverable bullets

**[8.4.1]** Unnumbered **`-`** bullets for narrative feature lists (Purpose, architecture components). Keep bullets **one line** when possible; use **`**bold**`** lead-ins for categories.

---

## 9. Code blocks and wire layouts

### 9.1 Fence languages

**[9.1.1]**

| Fence | Use |
|---|---|
| **` ```c `** | Structs, `#define`, function prototypes, normative API signatures, example `main` |
| **` ```text `** | Wire layouts, byte sequences, ASCII diagrams, range allocation charts, sequence diagrams |

**[9.1.2]** Do **not** use **` ```markdown `** inside normative specs. Avoid **` ```json `** / XML for on-wire formats; specs use **binary layouts**.

### 9.2 C API presentation

**[9.2.1]** Function declarations use **Allman-style** bracing (opening brace on its own line), **one parameter per line**, trailing **`//`** comments on parameters:

```c
cdnet_result_t
cdnet_client_connect
  (
  cdnet_client_t **OutClient,                       // out: new session handle
  const cdnet_client_config_t *Config              // connect-time configuration
  );
```

**[9.2.2]** After the fence, follow with **`[N.M.P]`** paragraphs: one-line **summary** (function name bolded), then **Rules —** numbered list for behavior, errors, threading, timeouts.

**[9.2.3]** **Opaque types** — forward-declare `struct _name` + typedef; full struct definitions only where layout is public and normative.

### 9.3 Wire structs

**[9.3.1]** C struct in **` ```c `** shows logical field order and types; comments use **`/* ... */`** for field semantics.

**[9.3.2]** Follow with **octet size** statements (**“occupies exactly **6** bytes”**), **little-endian** rule pointers, and **shall be 0 on transmit** for reserved fields.

### 9.4 Wire body layouts

**[9.4.1]** Message bodies use **` ```text `** pseudo-grammar:

```text
uint8_t   Status                     — CDSL_STATUS_OK (0) or error code (§16.1)
uint16_t  InterfaceCount             — number of Interface records that follow

For each Interface (InterfaceCount times):
  uint16_t  InterfaceId
  ...
```

**[9.4.2]** Conventions:

- Type name left-aligned, **`—`** em dash before description.
- **`For each X (Count times):`** for repeated blocks.
- Variable-length strings: **`uint8_t Len`** then **`char Name[Len]`** with **including NUL** stated.
- Point to status-code tables (**§16.1**) instead of duplicating enum prose.

### 9.5 Diagrams

**[9.5.1]** Architecture diagrams use **` ```text `** ASCII art (boxes, arrows). Sequence diagrams use the same: left/right roles, **`|---`**, **`|<--`**, message names in **CAPS** or **`function()`** as appropriate.

**[9.5.2]** Mermaid is **not** used in the current CDNet family; prefer plain **` ```text `** for universal diff-friendliness.

---

## 10. Tables

### 10.1 When to use tables

**[10.1.1]** Use markdown tables for:

- Revision history (**§0.1**)
- Registry assignments (value | `#define` | network | purpose)
- Request/response pairing (response | paired request)
- Checklist index (**Topic | Primary sections**)
- Design-goal ↔ checklist mapping

### 10.2 Format

**[10.2.1]** Header row, then **`|---|---|`** separator. Numeric values in backticks: **`| `0x0001` |`**.

**[10.2.2]** Place the paragraph id **on the line above** the table, not inside cells.

---

## 11. API and error-model documentation

### 11.1 Error types

**[11.1.1]** Document **one** result enumeration per library (**`cdnet_result_t`**, **`cdservice_result_t`**). Rules: **`OK == 0`**, failures **strictly negative**, **`get_last_error_message()`** for UTF-8 text.

**[11.1.2]** Layer specs **shall** state **separate** error enums when sitting on top of another library (**§2.0.26** pattern), with **same conventions** as the base.

### 11.2 Timeouts

**[11.2.1]** Centralize **bounded waits**: no infinite blocking; **`TimeoutMs`**, **`CDNET_TIMEOUT_DEFAULT`**, config struct defaults, and which APIs accept the default sentinel.

### 11.3 Callbacks and events

**[11.3.1]** Present **`typedef struct`** of function pointers, then **bullet per callback** with: when invoked, reentrancy, pointer lifetime, **shall not** invoke **X** from **Y**.

**[11.3.2]** Managed projections (**C#** `event`) get a dedicated **§** with **shall** mirror native semantics.

### 11.4 Complete header section

**[11.4.1]** Large specs include **Normative API Header (`*.h`)** — essentially the public header as a single fenced block plus per-group rules. Keeps implementers and agentic tools aligned on spelling.

---

## 12. Message and protocol catalogs

### 12.1 Section structure

**[12.1.1]** **Wire format** chapter defines headers, endianness, version constants, and **MessageType** / opcode ranges.

**[12.1.2]** **Messages** chapter uses **`### 11.N MESSAGE_NAME`** per message:

1. **Purpose.**
2. **Request body** — type constant, **`BodyLen`**, layout fence.
3. **Response body** — layout fence.
4. **Normative behavior** — status codes, broadcast vs unicast, correlation (**`RequestId`**), side effects (notifications).

### 12.2 Naming

**[12.2.1]** Message names are **UPPER_SNAKE** in prose and **`#define`**: **`LIST_INTERFACES`**, **`CDSL_MSG_CONNECT_CHANNEL_REQ`**.

**[12.2.2]** Distinguish **CDNet `Protocol`** (envelope) from **CDSL `MessageType`** (payload body); registry **§2.1** vs **§2.2** pattern.

---

## 13. Registries (authoring rules)

**[13.1.1]** Structure:

1. Purpose and coexistence rules (**§0**)
2. Field definition (size, endianness, pointer to base spec **§12**)
3. **Range allocation** (` ```text `)
4. **Rules for claiming** (numbered)
5. **Assigned values** tables per owner protocol
6. **C header fragment (normative names)**
7. **Coexistence checklist** for third-party protocols
8. **Revision history**

**[13.1.2]** When the base spec’s **§13** “suggested ranges” overlap the registry, include an **Overlap note** explaining why there is no wire conflict (internal vs application-visible bands, separate logical networks).

**[13.1.3]** Reserved bands use **`| — | — | *Reserved for …* |`** rows, not silent gaps.

---

## 14. Implementation reinforcement

### 14.1 Checklist section

**[14.1.1]** Large layer specs end with **Specification Summary and Implementation Reinforcement Checklist** (**§25** pattern):

- **`### 25.N Topic`** with **`☐`** items
- Each item cites authoritative **§** (and addendum/registry as needed)
- **Do Not Implement** list for v1 anti-patterns
- **Section index** table mapping topics → sections
- Optional mapping **Design goal §2** → checklist items

**[14.1.2]** Checklist items are **verification-oriented**, not alternate normative text; if a checkbox disagrees with an earlier **§**, the **§** wins.

### 14.2 Testing

**[14.2.1]** **Automated testing** **shall** name default harness (e.g. **virtual loopback**), what runs in CI without hardware, and optional hardware-tagged tests.

### 14.3 Build layout

**[14.3.1]** **Reference implementation** sections use **shall** for directory names, which binaries link which libraries, forbidden dependencies (**`cdnet.*` shall not contain CDSL**), and layer diagrams mirroring **§5.1** architecture.

---

## 15. Logging and diagnostics

**[15.1.1]** File logging specs define: line format, timestamp resolution, **module names** (normative list), header line on file open, optional **`LogCanFrames`**-style toggles, and **shall not** affect unrelated subsystems.

**[15.1.2]** List **API calls** and **internal events** as bullet log lines (e.g. **`- `cdnet_server_unregister_network()` (network id, result)`**).

---

## 16. Agentic and human collaboration

**[16.1.1]** Specs are written for **human maintainers** and **agentic AI** implementers: paragraph ids, complete header sketches, checklists, **fully commented** source requirements, and explicit **shall not** lists reduce ambiguity.

**[16.1.2]** Prefer **deterministic** naming (**network names**, **ID assignment**, **endianness**) over implementation-defined behavior; where implementation-defined is unavoidable, say so and bound test expectations.

**[16.1.3]** When extending a frozen baseline, **addendum + registry row + SERVICE_LAYER §** update together; never assign **`Protocol`** values only in prose.

---

## 17. Quick reference — new specification checklist

**[17.1.1]** ☐ Create **`PROJECT-SPEC.md`** with title suffix **`[0.0.1]`** and **§0.0.2** paragraph-id legend.

**[17.1.2]** ☐ Write **§0** metadata: type, scope, platforms, version axes, related docs, phases.

**[17.1.3]** ☐ Add **§0.1** revision table and semver rules.

**[17.1.4]** ☐ Use **`## 1.`–`## N.`** spine: Purpose, Goals, Non-Goals, Terminology, Architecture, …

**[17.1.5]** ☐ Run **`tools/number_spec_paragraphs.py`** (source in **§2.5**); verify no **`[S.U.P]`** on the same line as a **` ``` `** fence.

**[17.1.6]** ☐ Document wire format in **` ```c `** + **` ```text `**; state **little-endian** and sizes.

**[17.1.7]** ☐ Document API in Allman **` ```c `** + **Rules —** numbered lists.

**[17.1.8]** ☐ Create or update **PROTOCOL-REGISTRY.md** for numeric IDs.

**[17.1.9]** ☐ Add **implementation checklist** and **summary** for layer specs.

**[17.1.10]** ☐ Cross-link **CDNET-SPEC-ADDENDA*** when baseline API is extended.

**[17.1.11]** ☐ Confirm spec files are UTF-8 (no BOM); see **§1.6**.

---

## 18. Summary

**[18.0.1]** Andy’s specifications are **long, traceable, normative markdown** documents: every paragraph tagged **`[S.U.P]`**, **`§`** cross-references, **`shall`**/ **`must`** language, **` ```c `** / **` ```text `** wire and API artifacts, external **registries** for numbers, **addenda** for baseline extensions, and **checklists** that turn architecture into verifiable obligations. **`ANDY-CCODINGSTYLEGUIDE.md`** and **`ANDY-CSHARPCODINGSTYLEGUIDE.md`** cover how source code looks; **this guide** covers how the specification itself is structured.
