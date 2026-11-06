"""Checks for the report codes and the text tables (config/i18n/de.toml, en.toml).

Usage: python tools/i18n_check.py [codes] [tables] [phases] [literals [PATH ...]]
No argument runs every check over the default scopes. Exit code 1 with one line per finding.

  codes     every code in codes.def has report.<id> in both tables, placeholders equal the listed params
  tables    de and en have the same keys and placeholder names, no key is a leaf and a prefix
  phases    every `// i18n-ns:` switch id has phase.<ns>.<id> in both tables
  literals  no German words / non-ASCII in core, bindings and game string literals
"""

import re
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CODES_DEF = ROOT / "core" / "include" / "starsim" / "report" / "codes.def"
DE_TABLE = ROOT / "config" / "i18n" / "de.toml"
EN_TABLE = ROOT / "config" / "i18n" / "en.toml"

GERMAN_WORDS = frozenset(
    "der die das und nicht kein keine wird werden ist auf aus bei nach zum zur vom beim fehlt laden "
    "geladen grenze bereit abstieg landung rueckkehr wetter schatten ansicht bewege erkannt "
    "abgebrochen abgelehnt moeglich geplant treibstoff triebwerk".split()
)

CODE_RE = re.compile(r'STARSIM_CODE\(\s*(\w+)\s*,\s*"([^"]*)"\s*,\s*"([^"]*)"\s*\)')
PLACEHOLDER_RE = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)(?::[^}]*)?\}")
NS_RE = re.compile(r"//\s*i18n-ns:\s*([a-z][a-z0-9_]*)")
CASE_RE = re.compile(r'case\s+[\w:]+\s*:\s*return\s+"([^"]*)"\s*;')
WORD_RE = re.compile(r"[A-Za-zÄÖÜäöüß]+")


_STRIP_RE = re.compile(r'"(?:[^"\\\n]|\\.)*"|//[^\n]*|/\*.*?\*/', re.S)
CODE_USE_RE = re.compile(r"STARSIM_CODE\s*\(")


def _strip_comments(text):
    """Blank out // and /* */ comments (newlines kept); strings stay."""

    def repl(m):
        g = m.group(0)
        return g if g.startswith('"') else re.sub(r"[^\n]", " ", g)

    return _STRIP_RE.sub(repl, text)


def parse_codes_def(text, findings=None):
    """Return [(id, [param names])] for every STARSIM_CODE entry (comments ignored, entries may wrap lines).

    An occurrence that does not parse is appended to `findings` (when given)."""
    stripped = _strip_comments(text)
    out, parsed_at = [], set()
    for m in CODE_RE.finditer(stripped):
        parsed_at.add(m.start())
        out.append((m.group(2), [p for p in m.group(3).split(",") if p]))
    if findings is not None:
        for m in CODE_USE_RE.finditer(stripped):
            line_start = stripped.rfind("\n", 0, m.start()) + 1
            if m.start() in parsed_at or stripped[line_start : m.start()].lstrip().startswith("#"):
                continue
            findings.append(f"codes.def:{stripped.count(chr(10), 0, m.start()) + 1}: STARSIM_CODE entry does not parse")
    return out


def placeholders(text):
    return set(PLACEHOLDER_RE.findall(text))


def flatten(table, prefix=""):
    """Nested table -> {"a.b.c": value}. Quoted keys holding dots expand the same way."""
    out = {}
    for key, value in table.items():
        full = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            out.update(flatten(value, full))
        else:
            out[full] = value
    return out


def _prefix_conflicts(flat, label):
    keys = set(flat)
    found = []
    for key in sorted(keys):
        parts = key.split(".")
        for i in range(1, len(parts)):
            prefix = ".".join(parts[:i])
            if prefix in keys:
                found.append(f"{label}: key '{prefix}' is both a leaf and a prefix of '{key}'")
    return found


def check_tables(de, en):
    fde, fen = flatten(de), flatten(en)
    findings = _prefix_conflicts(fde, "de") + _prefix_conflicts(fen, "en")
    for key in sorted(set(fde) - set(fen)):
        findings.append(f"key '{key}' missing in en.toml")
    for key in sorted(set(fen) - set(fde)):
        findings.append(f"key '{key}' missing in de.toml")
    for key in sorted(set(fde) & set(fen)):
        a, b = placeholders(str(fde[key])), placeholders(str(fen[key]))
        if a != b:
            findings.append(f"key '{key}': placeholders differ (de {sorted(a)}, en {sorted(b)})")
        for label, value in (("de", fde[key]), ("en", fen[key])):
            if '"' in str(value):
                findings.append(f"key '{key}' ({label}): value contains a double quote")
    return findings


_TOML_TOKEN_RE = re.compile(r"\"(?:[^\"\\\n]|\\.)*\"|'[^'\n]*'|#[^\n]*")


def check_raw_toml(text, label):
    """Subset rules on the raw file text: no escapes in basic strings, no multi-line strings."""
    findings = []
    for line_no, line in enumerate(text.splitlines(), 1):
        if '"""' in line or "'''" in line:
            findings.append(f"{label}:{line_no}: multi-line string is not in the TomlLite subset")
            continue
        for m in _TOML_TOKEN_RE.finditer(line):
            if m.group(0).startswith('"') and "\\" in m.group(0):
                findings.append(f"{label}:{line_no}: escape sequence in value (not in the TomlLite subset)")
    return findings


def check_codes(codes, de, en):
    fde, fen = flatten(de), flatten(en)
    findings = []
    for code_id, params in codes:
        if code_id == "ok":
            continue
        key = f"report.{code_id}"
        for label, flat in (("de", fde), ("en", fen)):
            if key not in flat:
                findings.append(f"code '{code_id}': key '{key}' missing in {label}.toml")
            elif placeholders(str(flat[key])) != set(params):
                findings.append(
                    f"code '{code_id}': placeholders in {label}.toml {sorted(placeholders(str(flat[key])))}"
                    f" differ from params {sorted(params)}"
                )
    return findings


CASE_LINE_RE = re.compile(r'^\s*case\s+[\w:]+\s*:\s*return\s+"([^"]*)"\s*;\s*(//.*)?$')
_LITERAL_RE = re.compile(r"\"(?:[^\"\\\n]|\\.)*\"|'(?:[^'\\\n]|\\.)*'")


def _code_part(line):
    """The line without string/char literals and without a trailing // comment."""
    return _LITERAL_RE.sub('""', line).split("//")[0]


def parse_phase_ids(cpp_text, findings=None, path=""):
    """{ns: [ids]} for every `// i18n-ns: <ns>` switch (marker line to the closing brace).

    Every `case` line there must be `case X::Y: return "id";` on one line. Other case lines and a
    marker without any id are appended to `findings` when given."""
    result = {}
    lines = cpp_text.splitlines()
    for i, line in enumerate(lines):
        m = NS_RE.search(line)
        if not m:
            continue
        ns = m.group(1)
        depth, opened = 0, False
        ids = result.setdefault(ns, [])
        for k in range(i, len(lines)):
            later = lines[k]
            code = _code_part(later)
            for ch in code:
                if ch == "{":
                    depth += 1
                    opened = True
                elif ch == "}":
                    depth -= 1
            if re.match(r"\s*case\b", code):
                cm = CASE_LINE_RE.match(later)
                if cm is None:
                    if findings is not None:
                        findings.append(
                            f"{path}:{k + 1}: case line in i18n-ns '{ns}' is not `case X::Y: return \"id\";`"
                        )
                elif cm.group(1) not in ids:
                    ids.append(cm.group(1))
            if opened and depth <= 0:
                break
        if not ids and findings is not None:
            findings.append(f"{path}:{i + 1}: i18n-ns '{ns}' marker yields no phase ids")
    return result


def check_phases(phase_ids, de, en):
    fde, fen = flatten(de), flatten(en)
    findings = []
    for ns, ids in sorted(phase_ids.items()):
        for phase_id in ids:
            key = f"phase.{ns}.{phase_id}"
            for label, flat in (("de", fde), ("en", fen)):
                if key not in flat:
                    findings.append(f"phase '{ns}.{phase_id}': key '{key}' missing in {label}.toml")
    return findings


def _bad_literal(content):
    if any(ord(c) > 127 for c in content):
        return "non-ASCII"
    for word in WORD_RE.findall(content):
        if word.lower() in GERMAN_WORDS:
            return f"German word '{word}'"
    return None


RAW_RE = re.compile(r'R"([^()\\\s"]{0,16})\(')


def _quoted_end(text, i, quote, multiline):
    """Index of the closing quote of the literal opened at i (or of the newline/end that stops it)."""
    n = len(text)
    j = i + 1
    while j < n and text[j] != quote and (multiline or text[j] != "\n"):
        j += 2 if text[j] == "\\" else 1
    return min(j, n)


def _scan(text, path, comment_start, block_comments, triple_quotes, quotes, cpp=False, multiline=False):
    """Collect string literals outside comments; return findings for the bad ones."""
    lines = text.splitlines()
    findings = []
    i, n, line_no = 0, len(text), 1
    while i < n:
        c = text[i]
        if c == "\n":
            line_no += 1
            i += 1
        elif text.startswith(comment_start, i):
            while i < n and text[i] != "\n":
                i += 1
        elif block_comments and text.startswith("/*", i):
            end = text.find("*/", i + 2)
            end = n if end < 0 else end + 2
            line_no += text.count("\n", i, end)
            i = end
        elif triple_quotes and (text.startswith('"""', i) or text.startswith("'''", i)):
            tq = text[i : i + 3]
            end = text.find(tq, i + 3)
            end = n if end < 0 else end + 3
            content = text[i + 3 : max(i + 3, end - 3)]
            _report(findings, lines, path, line_no, content)
            line_no += text.count("\n", i, end)
            i = end
        elif cpp and c == "R" and (i == 0 or text[i - 1] in "uUL8" or not (text[i - 1].isalnum() or text[i - 1] == "_")) and RAW_RE.match(text, i):
            m = RAW_RE.match(text, i)
            close = ")" + m.group(1) + '"'
            end = text.find(close, m.end())
            end = n if end < 0 else end + len(close)
            _report(findings, lines, path, line_no, text[m.end() : max(m.end(), end - len(close))])
            line_no += text.count("\n", i, end)
            i = end
        elif cpp and c == "'":
            if i > 0 and text[i - 1].isalnum():  # digit separator 1'000
                i += 1
            else:  # char literal: a skipped token
                j = _quoted_end(text, i, "'", False)
                i = j + 1 if j < n and text[j] == "'" else j
        elif c in quotes:
            j = _quoted_end(text, i, c, multiline)
            _report(findings, lines, path, line_no, text[i + 1 : j])
            line_no += text.count("\n", i, j)
            i = j + 1 if j < n and text[j] == c else j
        else:
            i += 1
    return findings


def _report(findings, lines, path, line_no, content):
    reason = _bad_literal(content)
    if reason is None:
        return
    if 0 < line_no <= len(lines) and "i18n-ok" in lines[line_no - 1]:
        return
    findings.append(f"{path}:{line_no}: {reason} in literal \"{content[:60]}\"")


def scan_cpp(text, path):
    return _scan(text, path, "//", True, False, '"', cpp=True)


def scan_gd(text, path):
    return _scan(text, path, "#", False, True, "\"'")


def scan_tscn(text, path):
    """.tscn: `;` comments, strings may span lines (embedded shader code)."""
    return _scan(text, path, ";", False, False, '"', multiline=True)


def _is_test_file(path):
    rel = path.relative_to(ROOT) if path.is_relative_to(ROOT) else path
    return "tests" in rel.parts or path.name.startswith("test_")


def _literal_files(paths):
    files = []
    if paths:
        for p in paths:
            p = Path(p)
            if p.is_dir():
                # a directory is scanned like the default scope: test files and fixtures are skipped
                files += [f for f in sorted(p.rglob("*"))
                          if f.is_file() and not _is_test_file(f.relative_to(p))]
            else:
                files.append(p)
    else:
        for sub in ("core/src", "core/include", "bindings/src"):
            files += sorted((ROOT / sub).rglob("*"))
        for pattern in ("*.gd", "*.tscn"):
            files += [f for f in sorted((ROOT / "game").rglob(pattern)) if not _is_test_file(f)]
    return [f for f in files if f.is_file() and f.suffix in {".cpp", ".hpp", ".h", ".gd", ".tscn"}]


def check_literals(paths):
    findings = []
    for f in _literal_files(paths):
        text = f.read_text(encoding="utf-8", errors="replace")
        shown = f.relative_to(ROOT).as_posix() if f.is_relative_to(ROOT) else str(f)
        if f.suffix == ".gd":
            findings += scan_gd(text, shown)
        elif f.suffix == ".tscn":
            findings += scan_tscn(text, shown)
        else:
            findings += scan_cpp(text, shown)
    return findings


def _load_tables():
    for path in (DE_TABLE, EN_TABLE):
        if not path.exists():
            return None, None, [f"config/i18n/{path.name} missing"]
    with open(DE_TABLE, "rb") as f:
        de = tomllib.load(f)
    with open(EN_TABLE, "rb") as f:
        en = tomllib.load(f)
    return de, en, []


def main(argv):
    checks, paths = [], []
    for i, arg in enumerate(argv):
        if arg == "literals":
            checks.append(arg)
            paths = argv[i + 1 :]
            break
        if arg not in {"codes", "tables", "phases"}:
            print(f"unknown check '{arg}'")
            return 2
        checks.append(arg)
    if not checks:
        checks = ["codes", "tables", "phases", "literals"]

    findings = []
    de = en = None
    if any(c in checks for c in ("codes", "tables", "phases")):
        de, en, missing = _load_tables()
        findings += missing
    if de is not None:
        if "codes" in checks:
            codes = parse_codes_def(CODES_DEF.read_text(encoding="utf-8"), findings)
            findings += check_codes(codes, de, en)
        if "tables" in checks:
            findings += check_tables(de, en)
            for table in (DE_TABLE, EN_TABLE):
                findings += check_raw_toml(table.read_text(encoding="utf-8"), f"config/i18n/{table.name}")
        if "phases" in checks:
            merged = {}
            for f in sorted((ROOT / "core" / "src").rglob("*.cpp")):
                shown = f.relative_to(ROOT).as_posix()
                text = f.read_text(encoding="utf-8", errors="replace")
                for ns, ids in parse_phase_ids(text, findings, shown).items():
                    merged.setdefault(ns, [])
                    merged[ns] += [x for x in ids if x not in merged[ns]]
            findings += check_phases(merged, de, en)
    if "literals" in checks:
        findings += check_literals(paths)

    for line in findings:
        print(line)
    return 1 if findings else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
