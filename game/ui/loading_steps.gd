class_name LoadingSteps
extends RefCounted
## The startup loading screen's data: the boot steps reported so far (German texts), the one running now and a short
## dimmed history of the finished ones. Pure logic (no nodes), so it is unit-tested headless.

const HISTORY_MAX := 4  ## finished steps shown dimmed above the current one

var title := ""
var finished := false  ## once true, add() is ignored (the screen is gone for good)
var _texts: Array[String] = []
var _start_ms: Array[int] = []
var _end_ms := 0


## A new step starts at `now_ms`; the previous one ends. Empty text and a step after finish() are ignored.
## A repeated text (the same step reported twice) only keeps the first start.
func add(text: String, now_ms: int) -> void:
	if finished or text.is_empty() or (not _texts.is_empty() and _texts[-1] == text):
		return
	_texts.append(text)
	_start_ms.append(now_ms)


func finish(now_ms: int) -> void:
	if not finished:
		finished = true
		_end_ms = now_ms


func count() -> int:
	return _texts.size()


## The text of step `index` (0 = the first reported).
func text_at(index: int) -> String:
	return _texts[index]


## The running step ("" before the first).
func current() -> String:
	return "" if _texts.is_empty() else _texts[-1]


## The finished steps right before the current one, oldest first, at most HISTORY_MAX.
func history() -> Array[String]:
	var last := _texts.size() - 1
	var first := maxi(0, last - HISTORY_MAX)
	var out: Array[String] = []
	for i in range(first, last):
		out.append(_texts[i])
	return out


## Duration of step `index` in ms (the running one: until the end if finished, else until `now_ms`).
func duration_ms(index: int, now_ms: int) -> int:
	var end := _start_ms[index + 1] if index + 1 < _texts.size() else (_end_ms if finished else now_ms)
	return end - _start_ms[index]


## One log line per step, "+start s  duration s  text": the boot table printed when the screen is done.
func report_lines(now_ms: int) -> PackedStringArray:
	var lines := PackedStringArray()
	for i in _texts.size():
		lines.append("%+7.2f s  %6.2f s  %s" % [_start_ms[i] / 1000.0, duration_ms(i, now_ms) / 1000.0, _texts[i]])
	return lines
