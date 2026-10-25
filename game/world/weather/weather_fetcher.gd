class_name WeatherFetcher
extends Node
## HTTP downloads of the weather service (non-blocking HTTPRequest nodes on their own threads, optional byte range). The
## service talks to a fetcher only through `request()` and the `finished` signal, so tests inject a fake with the same
## two members.
##
## A file download goes to `<dest>.part` and is renamed to `dest` only after a complete success (HTTP 200, or 206 when a
## range was requested; the engine's body-size check; a non-empty file): a damaged or truncated file is never visible
## under its final name. `dest == ""` returns the body in the signal instead (small text: the GFS index, the ERA5
## .zattrs).
##
## There is no cap on the whole request (an ERA5 chunk is tens of MB, a slow line needs minutes): a transfer is given up
## only when it STALLS, i.e. no byte arrived for STALL_TIMEOUT_S (the connect phase counts as no bytes).

## id: the request's id; ok: complete success; http_code: the HTTP status (0: no response); body: the text bytes when
## the request had no destination file.
signal finished(id: int, ok: bool, http_code: int, body: PackedByteArray)

const STALL_TIMEOUT_S := 30.0  ## [call] a transfer without a single new byte for this long is given up
const DOWNLOAD_CHUNK_BYTES := 1 << 20  ## [call]

var _next_id := 1
var _active := {}  ## id -> {http: HTTPRequest, dest: String, ranged: bool, bytes: int, progress_s: float}


## Starts a download; returns its id at once. last_byte < 0: from first_byte to the end (first_byte < 0 too: all).
func request(url: String, first_byte: int, last_byte: int, dest: String) -> int:
	var id := _next_id
	_next_id += 1
	var http := HTTPRequest.new()
	http.use_threads = true  # the transfer must not run on the main thread
	http.timeout = 0.0  # no cap on the whole request; stalls are detected in _process
	http.download_chunk_size = DOWNLOAD_CHUNK_BYTES
	var part := ""
	if dest != "":
		DirAccess.make_dir_recursive_absolute(dest.get_base_dir())
		part = dest + ".part"
		http.download_file = part
	add_child(http)
	_active[id] = {"http": http, "dest": dest, "ranged": first_byte >= 0, "bytes": 0,
			"progress_s": Time.get_ticks_msec() / 1000.0}
	http.request_completed.connect(_on_completed.bind(id))
	var headers := PackedStringArray()
	if first_byte >= 0:
		headers.append(range_header(first_byte, last_byte))
	var err := http.request(url, headers)
	if err != OK:
		_on_completed.call_deferred(HTTPRequest.RESULT_CANT_CONNECT, 0, PackedStringArray(), PackedByteArray(), id)
	return id


## The Range header line of a request (last_byte < 0: open ended).
static func range_header(first_byte: int, last_byte: int) -> String:
	return "Range: bytes=%d-%s" % [first_byte, str(last_byte) if last_byte >= 0 else ""]


## Is the answer a complete success? A ranged request is accepted only with 206 (a 200 full-file answer would be the
## wrong bytes), an unranged one only with 200.
static func accepts(result: int, code: int, ranged: bool) -> bool:
	if result != HTTPRequest.RESULT_SUCCESS:
		return false
	return code == 206 if ranged else code == 200


## Finishes a file download: on acceptance and a non-empty `<dest>.part` the part becomes `dest`; otherwise the part
## is deleted. Returns whether `dest` now holds the complete file.
static func settle_file(result: int, code: int, ranged: bool, dest: String) -> bool:
	var part := dest + ".part"
	var ok := accepts(result, code, ranged) and FileAccess.file_exists(part)
	if ok:
		var f := FileAccess.open(part, FileAccess.READ)
		var size := 0
		if f != null:
			size = f.get_length()
			f.close()
		ok = size > 0
	if ok:
		if FileAccess.file_exists(dest):
			DirAccess.remove_absolute(dest)
		ok = DirAccess.rename_absolute(part, dest) == OK
	if not ok and FileAccess.file_exists(part):
		DirAccess.remove_absolute(part)
	return ok


func _process(_delta: float) -> void:
	var now := Time.get_ticks_msec() / 1000.0
	for id: int in _active.keys():
		var entry: Dictionary = _active[id]
		var http: HTTPRequest = entry["http"]
		var bytes := http.get_downloaded_bytes()
		if bytes != int(entry["bytes"]):
			entry["bytes"] = bytes
			entry["progress_s"] = now
		elif now - float(entry["progress_s"]) >= STALL_TIMEOUT_S:
			http.cancel_request()  # closes the part file; request_completed is not emitted
			_on_completed(HTTPRequest.RESULT_TIMEOUT, 0, PackedStringArray(), PackedByteArray(), id)


func _on_completed(result: int, code: int, _headers: PackedStringArray, body: PackedByteArray, id: int) -> void:
	var entry: Dictionary = _active.get(id, {})
	if entry.is_empty():
		return
	_active.erase(id)
	var http: HTTPRequest = entry["http"]
	var dest: String = entry["dest"]
	var ranged: bool = entry["ranged"]
	var ok: bool
	if dest != "":
		ok = settle_file(result, code, ranged, dest)
	else:
		ok = accepts(result, code, ranged)
	http.queue_free()
	finished.emit(id, ok, code, body if dest == "" and ok else PackedByteArray())
