"""Tests of tools/i18n_check.py. Run: python -m pytest tools/test_i18n_check.py -q"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import i18n_check as c  # noqa: E402


def test_parse_codes_def():
    text = '// comment\nSTARSIM_CODE(lunar_site_too_steep, "lunar.site_too_steep", "slope,limit")\n'
    assert c.parse_codes_def(text) == [("lunar.site_too_steep", ["slope", "limit"])]


def test_parse_codes_def_no_params():
    assert c.parse_codes_def('STARSIM_CODE(ok, "ok", "")') == [("ok", [])]


def test_placeholders():
    assert c.placeholders("zu steil: {slope:deg1} (Grenze {limit})") == {"slope", "limit"}


def test_flatten_nested_and_sections():
    table = {"report": {"lunar": {"site": "a"}, "x": "b"}, "ui": {"title": "t"}}
    assert c.flatten(table) == {"report.lunar.site": "a", "report.x": "b", "ui.title": "t"}


def test_key_missing_in_en_fails():
    de = {"report": {"a": "x", "b": "y"}}
    en = {"report": {"a": "x"}}
    assert any("report.b" in f and "en.toml" in f for f in c.check_tables(de, en))
    assert c.check_tables(de, de) == []


def test_placeholder_mismatch_fails():
    de = {"report": {"a": "{x} und {y}"}}
    en = {"report": {"a": "{x}"}}
    assert any("placeholders differ" in f for f in c.check_tables(de, en))


def test_code_without_text_fails():
    codes = [("lunar.site_too_steep", ["slope"])]
    assert any("missing in de.toml" in f for f in c.check_codes(codes, {}, {}))
    good = {"report": {"lunar": {"site_too_steep": "steil {slope:deg1}"}}}
    assert c.check_codes(codes, good, good) == []
    wrong = {"report": {"lunar": {"site_too_steep": "steil"}}}
    assert c.check_codes(codes, wrong, good)


def test_leaf_prefix_conflict_fails():
    bad = {"report": {"a": "x", "a.b": "y"}}
    assert any("leaf and a prefix" in f for f in c.check_tables(bad, bad))


def test_scan_cpp():
    src = (
        'auto a = "Höhe";\n'
        'auto b = "Grenze erreicht";\n'
        "// Grenze\n"
        'auto c = "engine";\n'
        'auto d = "Grenze"; // i18n-ok\n'
        "/* Wetter */ auto e = 1;\n"
    )
    findings = c.scan_cpp(src, "x.cpp")
    assert len(findings) == 2
    assert "x.cpp:1" in findings[0] and "x.cpp:2" in findings[1]


def test_scan_gd():
    src = 'var a = "Höhe"\n# Höhe\nvar b = "engine"\n'
    findings = c.scan_gd(src, "x.gd")
    assert len(findings) == 1 and "x.gd:1" in findings[0]


def test_parse_phase_ids():
    src = (
        "const char* id_of(DockingPhase p) {  // i18n-ns: docking_phase\n"
        "    switch (p) {\n"
        '        case DockingPhase::Approach: return "approach";\n'
        '        case DockingPhase::SoftCapture: return "soft_capture";\n'
        "    }\n"
        '    return "approach";\n'
        "}\n"
        'const char* other(int) { return "no"; }\n'
    )
    assert c.parse_phase_ids(src) == {"docking_phase": ["approach", "soft_capture"]}


def test_phase_id_without_key_fails():
    ids = {"docking_phase": ["approach", "soft_capture"]}
    tables = {"phase": {"docking_phase": {"approach": "Anflug"}}}
    findings = c.check_phases(ids, tables, tables)
    assert any("phase.docking_phase.soft_capture" in f for f in findings)
    assert not any("phase.docking_phase.approach" in f for f in findings)


def test_scan_cpp_char_literal_and_continuation_keep_line_numbers():
    src = "char q = '\"';\nauto a = \"Grenze\";\n#define X \\\n  1\nauto b = \"Grenze\"; // i18n-ok\nauto c = \"Wetter\";\n"
    findings = c.scan_cpp(src, "x.cpp")
    assert [f.split(":")[1] for f in findings] == ["2", "6"]


def test_scan_cpp_unterminated_string_keeps_line_numbers():
    findings = c.scan_cpp('auto a = "oops;\nauto b = "Grenze";\n', "x.cpp")
    assert len(findings) == 1 and "x.cpp:2" in findings[0]


def test_scan_cpp_raw_string():
    src = 'auto a = R"(say "hi" und nicht)";\nauto b = R"x(plain)x";\nauto d = "Grenze";\n'
    findings = c.scan_cpp(src, "x.cpp")
    assert len(findings) == 2 and "x.cpp:1" in findings[0] and "x.cpp:3" in findings[1]


def test_scan_tscn_multiline_string_and_semicolon_comment():
    src = 'a = "line1\nline2"\n; "Höhe"\nb = "Höhe"\n'
    findings = c.scan_tscn(src, "x.tscn")
    assert len(findings) == 1 and "x.tscn:4" in findings[0]


def test_scan_gd_single_triple_quote():
    assert len(c.scan_gd("var a = '''Höhe'''\n", "x.gd")) == 1


def test_parse_codes_def_wrapped_and_block_comment():
    text = '/* STARSIM_CODE(old, "old", "") */\nSTARSIM_CODE(a_b,\n    "a.b",\n    "x")\n'
    findings = []
    assert c.parse_codes_def(text, findings) == [("a.b", ["x"])]
    assert findings == []


def test_parse_codes_def_unparsable_is_finding():
    findings = []
    assert c.parse_codes_def('STARSIM_CODE(a, "a")\n', findings) == []
    assert len(findings) == 1 and "does not parse" in findings[0]


def test_parse_phase_ids_wrapped_case_is_finding():
    src = (
        "const char* id_of(P p) {  // i18n-ns: ns\n"
        "    switch (p) {\n"
        '        case P::A: return "a";\n'
        "        case P::B:\n"
        '            return "b";\n'
        '        case P::C: { return "c"; }\n'
        '        default: return "a}";\n'
        "    }\n"
        "}\n"
    )
    findings = []
    assert c.parse_phase_ids(src, findings, "x.cpp") == {"ns": ["a"]}
    assert len(findings) == 2 and "x.cpp:4" in findings[0] and "x.cpp:6" in findings[1]


def test_parse_phase_ids_marker_without_ids_is_finding():
    findings = []
    c.parse_phase_ids("int f() {  // i18n-ns: empty\n  return 1;\n}\n", findings, "x.cpp")
    assert any("yields no phase ids" in f for f in findings)


def test_raw_toml_escapes_and_multiline_fail():
    assert c.check_raw_toml('a = "ok" # "x\\n"\nb = \'C:\\p\'\n', "de") == []
    assert len(c.check_raw_toml('a = "l\\n"\n', "de")) == 1
    assert c.check_raw_toml('a = """\nx"""\n', "de")
