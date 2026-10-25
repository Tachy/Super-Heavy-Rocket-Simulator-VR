class_name WeatherFetcher
extends Node
## HTTP downloads of the weather service (non-blocking HTTPRequest nodes, optional byte range). The service talks to a
## fetcher only through `request()` and the `finished` signal, so tests inject a fake with the same two members.
##
## A file download goes to `<dest>.part` and is renamed to `dest` only after a complete success (HTTP 200 / 206, the
## engine's body-size check, a non-empty file): a damaged or truncated file is never visible under its final name.
## `dest == ""` returns the body in the signal instead (small text: the GFS index, the ERA5 .zattrs).

## id: the request's id; ok: complete success; http_code: the HTTP status (0: no response); body: the text bytes when
## the request had no destination file.
signal finished(id: int, ok: bool, http_code: int, body: PackedByteArray)

const CONNECT_TIMEOUT_S := 30.0  ## [call] a stalled transfer is given up after this long without finishing
const DOWNLOAD_CHUNK_BYTES := 1 << 20  ## [call]

var _next_id := 1
var _active := {}  ## id -> {http: HTTPRequest, dest: String}


## Starts a download; returns its id at once. last_byte < 0: from first_byte to the end (first_byte < 0 too: all).
func request(url: String, first_byte: int, last_byte: int, dest: String) -> int:
	var id := _next_id
	_next_id += 1
	var http := HTTPRequest.new()
	http.timeout = CONNECT_TIMEOUT_S
	http.download_chunk_size = DOWNLOAD_CHUNK_BYTES
	var part := ""
	if dest != "":
		DirAccess.make_dir_recursive_absolute(dest.get_base_dir())
		part = dest + ".part"
		http.download_file = part
	add_child(http)
	_active[id] = {"http": http, "dest": dest}
	http.request_completed.connect(_on_completed.bind(id))
	var headers := PackedStringArray()
	if first_byte >= 0:
		headers.append("Range: bytes=%d-%s" % [first_byte, str(last_byte) if last_byte >= 0 else ""])
	var err := http.request(url, headers)
	if err != OK:
		_on_completed.call_deferred(HTTPRequest.RESULT_CANT_CONNECT, 0, PackedStringArray(), PackedByteArray(), id)
	return id


func _on_completed(result: int, code: int, _headers: PackedStringArray, body: PackedByteArray, id: int) -> void:
	var entry: Dictionary = _active.get(id, {})
	if entry.is_empty():
		return
	_active.erase(id)
	var http: HTTPRequest = entry["http"]
	var dest: String = entry["dest"]
	var ok: bool = result == HTTPRequest.RESULT_SUCCESS and (code == 200 or code == 206)
	if dest != "":
		var part := dest + ".part"
		if ok and FileAccess.file_exists(part):
			var size := 0
			var f := FileAccess.open(part, FileAccess.READ)
			if f != null:
				size = f.get_length()
				f.close()
			ok = size > 0 and DirAccess.rename_absolute(part, dest) == OK
		else:
			ok = false
		if not ok and FileAccess.file_exists(part):
			DirAccess.remove_absolute(part)
	http.queue_free()
	finished.emit(id, ok, code, body if dest == "" and ok else PackedByteArray())
