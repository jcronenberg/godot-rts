extends Node2D

## Replays one trace dumped by the movement quality harness:
##
##     cd rust && cargo run --release --example quality -- --trace door_funnel_200
##
## which writes `rust/target/quality/trace_<scenario>.json`. Set [member
## trace_path] to it and run the scene. When a score drops we want to *watch*
## the run, not read the number, and because the sim is headless and
## deterministic, this is the exact run that produced the number, not a
## re-enactment of it.
##
## Space plays/pauses, left/right step a tick (held: scrub), up/down change
## speed, Home rewinds, P toggles paths.
##
## Traces are one JSON object per tick per unit, so they scale with
## `units * ticks`: `concave_trap` is 3 MB, `door_funnel_200` is 67 MB and takes
## a while to parse. The default is a small one on purpose; point [member
## trace_path] at a bigger scenario once you know the scene works.

## Per-team unit color, indexed by team id (wraps via modulo). Matches
## `sim_test.gd` so a trace looks like the live view.
const TEAM_COLORS: Array[Color] = [
	Color(0.3, 0.6, 1.0),
	Color(1.0, 0.35, 0.3),
	Color(0.4, 0.9, 0.4),
	Color(0.9, 0.8, 0.3),
]

## Ring drawn around a unit for the first flag that applies, in this order
## (GDScript dictionaries iterate in insertion order, so the order is the
## priority). Stuck states outrank settled ones: a unit that is both parked and
## stalling is interesting for the stall.
const FLAG_COLORS := {
	"stall": Color(1.0, 0.2, 0.2),
	"ally_stall": Color(1.0, 0.6, 0.1),
	"hold": Color(1.0, 0.9, 0.2),
	"engaged": Color(1.0, 1.0, 1.0),
	"parked": Color(0.3, 1.0, 0.4),
}

## `_flag_color` result for a unit with nothing worth ringing.
const NO_FLAG := Color(0, 0, 0, 0)

@export_file("*.json") var trace_path: String = "res://rust/target/quality/trace_concave_trap.json"
## Trace ticks replayed per second at speed 1. The harness runs the sim at 30 Hz.
@export var tick_rate: float = 30.0
@export var show_paths: bool = true

@onready var _camera: RtsCamera = $Camera2D

var _scenario: String = ""
## `[{tick, segments}]`, each the complete wall set from that tick on. More
## than one entry means the scenario added or removed an obstacle mid-run.
var _wall_events: Array = []
var _ticks: Array = []
var _cursor: float = 0.0
var _playing: bool = true
var _speed: float = 1.0
var _label: Label


func _ready() -> void:
	_label = Label.new()
	_label.position = Vector2(8, 8)
	_label.add_theme_color_override("font_color", Color.WHITE)
	var ui := CanvasLayer.new()
	ui.add_child(_label)
	add_child(ui)
	_load(trace_path)


func _load(path: String) -> void:
	var text := FileAccess.get_file_as_string(path)
	if text.is_empty():
		push_error("quality_replay: cannot read %s (run the harness with --trace first)" % path)
		return
	var data: Variant = JSON.parse_string(text)
	if typeof(data) != TYPE_DICTIONARY:
		push_error("quality_replay: %s is not a trace" % path)
		return
	_scenario = data.get("scenario", "?")
	_wall_events = data.get("wall_events", [])
	_ticks = data.get("ticks", [])
	if _wall_events.is_empty():
		# Drop the frames too: `_draw` indexes `_wall_events[0]`, so a
		# half-loaded trace would crash on the first redraw rather than
		# showing the "no trace loaded" label.
		_ticks = []
		push_error("quality_replay: %s has no wall_events (regenerate it with --trace)" % path)
		return
	_cursor = 0.0
	_frame_camera()


## Fit the whole map on screen, so a trace from any scenario opens usable
## without touching the camera first. Framed over every wall the run ever has,
## not just the opening set, so a dropped building cannot land off-screen.
func _frame_camera() -> void:
	var first: Array = _wall_events[0]["segments"]
	if first.is_empty():
		return
	var rect := Rect2(Vector2(first[0][0], first[0][1]), Vector2.ZERO)
	for ev in _wall_events:
		for w in ev["segments"]:
			rect = rect.expand(Vector2(w[0], w[1])).expand(Vector2(w[2], w[3]))
	_camera.global_position = rect.get_center()
	var view := get_viewport_rect().size
	var fit: float = minf(view.x / maxf(rect.size.x, 1.0), view.y / maxf(rect.size.y, 1.0))
	var z: float = clampf(fit * 0.9, _camera.min_zoom, _camera.max_zoom)
	_camera.zoom = Vector2(z, z)


func _process(delta: float) -> void:
	if _playing and not _ticks.is_empty():
		_cursor = clampf(_cursor + delta * tick_rate * _speed, 0.0, _ticks.size() - 1.0)
		if _cursor >= _ticks.size() - 1.0:
			_playing = false
	if Input.is_action_pressed("ui_right"):
		_seek(1.0)
	elif Input.is_action_pressed("ui_left"):
		_seek(-1.0)
	_update_label()
	queue_redraw()


func _unhandled_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.is_pressed() or key.is_echo():
		return
	match key.keycode:
		KEY_SPACE:
			# Replaying from the end would sit on the last frame; rewind first.
			if not _playing and _cursor >= _ticks.size() - 1.0:
				_cursor = 0.0
			_playing = not _playing
		KEY_UP:
			_speed = minf(_speed * 2.0, 32.0)
		KEY_DOWN:
			_speed = maxf(_speed * 0.5, 0.0625)
		KEY_HOME:
			_cursor = 0.0
		KEY_P:
			show_paths = not show_paths
		_:
			return
	get_viewport().set_input_as_handled()


func _seek(ticks: float) -> void:
	_playing = false
	_cursor = clampf(_cursor + ticks, 0.0, maxf(_ticks.size() - 1.0, 0.0))


func _update_label() -> void:
	if _ticks.is_empty():
		_label.text = "no trace loaded; see %s" % trace_path
		return
	var frame: Dictionary = _ticks[int(_cursor)]
	_label.text = "%s   tick %d  (%d/%d)   x%.2f %s\nspace play/pause · ←/→ step · ↑/↓ speed · home rewind · p paths" % [
		_scenario, int(frame.get("tick", 0)), int(_cursor) + 1, _ticks.size(),
		_speed, "playing" if _playing else "paused",
	]


## The wall set in force at `tick`. Events are few (one, plus one per obstacle
## change) and in order, so a scan costs nothing.
func _walls_at(tick: int) -> Array:
	var segs: Array = _wall_events[0]["segments"]
	for ev in _wall_events:
		if int(ev["tick"]) > tick:
			break
		segs = ev["segments"]
	return segs


func _draw() -> void:
	if _ticks.is_empty():
		return
	var frame: Dictionary = _ticks[int(_cursor)]
	for w in _walls_at(int(frame.get("tick", 0))):
		draw_line(Vector2(w[0], w[1]), Vector2(w[2], w[3]), Color(0.8, 0.8, 0.85), 1.0)
	var units: Array = frame.get("units", [])
	if show_paths:
		for u in units:
			var path: Array = u.get("path", [])
			if path.size() < 1:
				continue
			var line := PackedVector2Array()
			line.append(Vector2(u["x"], u["y"]))
			for p in path:
				line.append(Vector2(p[0], p[1]))
			draw_polyline(line, Color(1, 1, 0, 0.25), 1.0)
	for u in units:
		var pos := Vector2(u["x"], u["y"])
		var r: float = u["r"]
		draw_circle(pos, r, TEAM_COLORS[int(u["team"]) % TEAM_COLORS.size()])
		var flag := _flag_color(u)
		if flag.a > 0.0:
			draw_arc(pos, r + 1.5, 0, TAU, 16, flag, 1.5)


## First flag that applies, or [constant NO_FLAG]. The trace carries booleans
## for latched states and tick counters for the stuck ones; both read as "on".
func _flag_color(unit: Dictionary) -> Color:
	for key in FLAG_COLORS:
		var v: Variant = unit.get(key)
		var on := bool(v) if typeof(v) == TYPE_BOOL else int(v) > 0
		if on:
			return FLAG_COLORS[key]
	return NO_FLAG
