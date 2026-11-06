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


def parse_codes_def(text):
    """Return [(id, [param names])] for every STARSIM_CODE line (comment lines are skipped)."""
    out = []
    for line in text.splitlines():
        if line.lstrip().startswith("//"):
            continue
        m = CODE_RE.search(line)
        if m:
            params = [p for p in m.group(3).split(",") if p]
            out.append((m.group(2), params))
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


def parse_phase_ids(cpp_text):
    """{ns: [ids]} for every `// i18n-ns: <ns>` switch (marker line to the closing brace)."""
    result = {}
    lines = cpp_text.splitlines()
    for i, line in enumerate(lines):
        m = NS_RE.search(line)
        if not m:
            continue
        ns = m.group(1)
        depth, opened = 0, False
        ids = result.setdefault(ns, [])
        for later in lines[i:]:
            code = later.split("//")[0] if "//" in later and '"' not in later.split("//")[0] else later
            for ch in code:
                if ch == "{":
                    depth += 1
                    opened = True
                elif ch == "}":
                    depth -= 1
            for cm in CASE_RE.finditer(later):
                if cm.group(1) not in ids:
                    ids.append(cm.group(1))
            if opened and depth <= 0:
                break
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


def _scan(text, path, comment_start, block_comments, triple_quotes, quotes):
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
        elif triple_quotes and text.startswith('"""', i):
            end = text.find('"""', i + 3)
            end = n if end < 0 else end + 3
            content = text[i + 3 : max(i + 3, end - 3)]
            _report(findings, lines, path, line_no, content)
            line_no += text.count("\n", i, end)
            i = end
        elif c in quotes:
            j = i + 1
            while j < n and text[j] != c and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            _report(findings, lines, path, line_no, text[i + 1 : min(j, n)])
            i = j + 1
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
    return _scan(text, path, "//", True, False, '"')


def scan_gd(text, path):
    return _scan(text, path, "#", False, True, "\"'")


def _is_test_file(path):
    return "tests" in path.parts or path.name.startswith("test_")


def _literal_files(paths):
    files = []
    if paths:
        for p in paths:
            p = Path(p)
            if p.is_dir():
                files += [f for f in sorted(p.rglob("*")) if f.is_file()]
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
        findings += scan_gd(text, shown) if f.suffix in {".gd", ".tscn"} else scan_cpp(text, shown)
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
            findings += check_codes(parse_codes_def(CODES_DEF.read_text(encoding="utf-8")), de, en)
        if "tables" in checks:
            findings += check_tables(de, en)
        if "phases" in checks:
            merged = {}
            for f in sorted((ROOT / "core" / "src").rglob("*.cpp")):
                for ns, ids in parse_phase_ids(f.read_text(encoding="utf-8", errors="replace")).items():
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
