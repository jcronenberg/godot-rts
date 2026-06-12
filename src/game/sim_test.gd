extends Node2D

## Test scene for the Rust sim thread: spawn units, drag-select,
## click-to-move with instant ack, building placement, debug overlay.

const BUILDING_SIZE := Vector2(20, 20)
const ACK_DURATION: float = 0.4
const DRAG_THRESHOLD: float = 5.0

@export var unit_radius: float = 5.0
@export var unit_speed: float = 60.0

## Level source of truth, editable in-editor; doubles as the runtime debug
## view of the sim's live navmesh (hidden until the overlay is toggled on).
@onready var _editor: DelaunayEditorNode = $DelaunayEditorNode

var _sim: Simulation
var _alpha: float = 0.0
var _ids: PackedInt64Array
var _positions: PackedVector2Array
var _radii: PackedFloat32Array
var _selected: Dictionary = {}  # unit id -> true

var _dragging: bool = false
var _drag_start: Vector2
var _drag_end: Vector2

var _ack_pos: Vector2
var _ack_time: float = -1.0

var _buildings: Dictionary = {}  # obstacle id -> Rect2
var _build_mode: bool = false
var _overlay: bool = false
var _mesh_version: int = -1
var _paths: Array[PackedVector2Array] = []

var _stats_label: Label
var _graphs: VBoxContainer
var _graph_fps: PerfGraph
var _graph_frame: PerfGraph
var _graph_step: PerfGraph
var _stats_timer: float = 0.0
var _frame_ms_acc: float = 0.0
var _frame_count: int = 0
var _last_stat_tick: int = -1


## Scrolling history graph for one metric.
class PerfGraph:
	extends Control

	const SAMPLES := 150

	var title: String
	var unit: String
	var color: Color
	var values := PackedFloat32Array()

	func _init(p_title: String, p_unit: String, p_color: Color) -> void:
		title = p_title
		unit = p_unit
		color = p_color
		custom_minimum_size = Vector2(240, 52)

	func push(value: float) -> void:
		values.append(value)
		if values.size() > SAMPLES:
			values = values.slice(values.size() - SAMPLES)
		if is_visible_in_tree():
			queue_redraw()

	func _draw() -> void:
		draw_rect(Rect2(Vector2.ZERO, size), Color(0, 0, 0, 0.55))
		if values.is_empty():
			return
		var vmax := 0.0001
		for v in values:
			vmax = maxf(vmax, v)
		var pts := PackedVector2Array()
		pts.resize(values.size())
		for i in values.size():
			pts[i] = Vector2(
				size.x * i / float(SAMPLES - 1),
				size.y - size.y * 0.85 * values[i] / vmax
			)
		if pts.size() >= 2:
			draw_polyline(pts, color, 1.0)
		var text := "%s %.2f%s (max %.2f)" % [title, values[-1], unit, vmax]
		draw_string(
			get_theme_default_font(), Vector2(4, 12), text,
			HORIZONTAL_ALIGNMENT_LEFT, -1, 10, Color.WHITE
		)


func _ready() -> void:
	_editor.visible = false
	_sim = Simulation.new()
	add_child(_sim)
	_sim.load_map(_editor.points, _editor.constraints, 42)
	_build_debug_ui()


func _process(delta: float) -> void:
	_alpha = _sim.poll(delta)
	_ids = _sim.get_unit_ids()
	_positions = _sim.get_positions(_alpha)
	_radii = _sim.get_radii()
	_prune_selection()
	if _overlay:
		_refresh_overlay()
	_update_stats(delta)
	queue_redraw()


func _update_stats(delta: float) -> void:
	_frame_ms_acc += delta * 1000.0
	_frame_count += 1
	_graph_frame.push(delta * 1000.0)
	var tick := _sim.tick()
	if tick != _last_stat_tick:
		_last_stat_tick = tick
		_graph_step.push(_sim.step_ms())
	# Refresh the text (and the slow FPS graph) a few times per second.
	_stats_timer += delta
	if _stats_timer < 0.25:
		return
	var fps := Engine.get_frames_per_second()
	var frame_ms := _frame_ms_acc / _frame_count
	_stats_label.text = (
		"%d fps | frame %.2f ms | sim step %.3f ms | %d units"
		% [fps, frame_ms, _sim.step_ms(), _ids.size()]
	)
	_graph_fps.push(fps)
	_stats_timer = 0.0
	_frame_ms_acc = 0.0
	_frame_count = 0


func _prune_selection() -> void:
	var alive := {}
	for id in _ids:
		alive[id] = true
	for id in _selected.keys():
		if not alive.has(id):
			_selected.erase(id)


func _refresh_overlay() -> void:
	if _sim.nav_version() != _mesh_version:
		_sim.request_mesh_dump()
		var dump: Dictionary = _sim.take_mesh_dump()
		if not dump.is_empty():
			_mesh_version = dump["version"]
			_editor.show_mesh_dump(dump["points"], dump["indices"], dump["edges"])
	_paths.assign(_sim.get_unit_paths())


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var world := get_global_mouse_position()
		match event.button_index:
			MOUSE_BUTTON_LEFT:
				if event.is_pressed():
					if _build_mode:
						_place_building(world)
					else:
						_dragging = true
						_drag_start = world
						_drag_end = world
				elif _dragging:
					_dragging = false
					_select(world)
			MOUSE_BUTTON_RIGHT:
				if not event.is_pressed():
					return
				if _build_mode:
					_remove_building_at(world)
				elif not _selected.is_empty():
					_move_selected(world)
	elif event is InputEventMouseMotion and _dragging:
		_drag_end = get_global_mouse_position()
	elif event is InputEventKey and event.is_pressed() and event.keycode == KEY_U:
		_sim.spawn_unit(get_global_mouse_position(), unit_radius, unit_speed)


func _select(world: Vector2) -> void:
	_selected.clear()
	if _drag_start.distance_to(_drag_end) < DRAG_THRESHOLD:
		# Plain click: pick the topmost unit under the cursor.
		for i in _ids.size():
			if world.distance_to(_positions[i]) <= _radii[i] + 2.0:
				_selected[_ids[i]] = true
				return
		return
	var rect := Rect2(_drag_start, Vector2.ZERO).expand(_drag_end)
	for i in _ids.size():
		if rect.has_point(_positions[i]):
			_selected[_ids[i]] = true


func _move_selected(goal: Vector2) -> void:
	var ids := PackedInt64Array()
	for id in _selected.keys():
		ids.append(id)
	_sim.move_units(ids, goal)
	# Instant view-side ack, independent of sim latency.
	_ack_pos = goal
	_ack_time = ACK_DURATION


func _place_building(center: Vector2) -> void:
	var half := BUILDING_SIZE / 2.0
	var box := PackedVector2Array(
		[
			center + Vector2(-half.x, -half.y),
			center + Vector2(half.x, -half.y),
			center + Vector2(half.x, half.y),
			center + Vector2(-half.x, half.y),
		]
	)
	var id := _sim.add_obstacle(box)
	if id != -1:
		_buildings[id] = Rect2(center - half, BUILDING_SIZE)


func _remove_building_at(pos: Vector2) -> void:
	for id in _buildings.keys():
		if (_buildings[id] as Rect2).has_point(pos):
			_sim.remove_obstacle(id)
			_buildings.erase(id)
			return


func _draw() -> void:
	if _overlay:
		for path in _paths:
			if path.size() >= 2:
				draw_polyline(path, Color(1, 1, 0, 0.6), 1.0)

	for id in _buildings:
		draw_rect(_buildings[id], Color(0.5, 0.3, 0.2))

	for i in _ids.size():
		var pos := _positions[i]
		if _selected.has(_ids[i]):
			draw_arc(pos, _radii[i] + 2.0, 0, TAU, 24, Color.GREEN, 1.5)
		draw_circle(pos, _radii[i], Color(0.3, 0.6, 1.0))

	if _dragging:
		var rect := Rect2(_drag_start, Vector2.ZERO).expand(_drag_end)
		draw_rect(rect, Color(0.4, 1.0, 0.4, 0.15))
		draw_rect(rect, Color(0.4, 1.0, 0.4), false, 1.0)

	if _ack_time > 0.0:
		_ack_time -= get_process_delta_time()
		var t := _ack_time / ACK_DURATION
		draw_arc(_ack_pos, 4.0 + 8.0 * t, 0, TAU, 16, Color(0.2, 1.0, 0.2, t), 2.0)


func _build_debug_ui() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	var panel := PanelContainer.new()
	panel.position = Vector2(8, 8)
	layer.add_child(panel)
	var col := VBoxContainer.new()
	panel.add_child(col)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 8)
	col.add_child(row)

	var hint := Label.new()
	hint.text = "U: spawn unit | drag: select | RMB: move"
	row.add_child(hint)

	var build := Button.new()
	build.toggle_mode = true
	build.text = "Place building"
	build.tooltip_text = "While on, LMB places a building, RMB removes one"
	build.toggled.connect(func(on: bool) -> void: _build_mode = on)
	row.add_child(build)

	var overlay := Button.new()
	overlay.toggle_mode = true
	overlay.text = "Debug overlay"
	overlay.toggled.connect(
		func(on: bool) -> void:
			_overlay = on
			_mesh_version = -1  # force a fresh dump on re-toggle
			_paths = []
			_editor.visible = on
			_sim.set_debug_overlay(on)
	)
	row.add_child(overlay)

	var pause := Button.new()
	pause.toggle_mode = true
	pause.text = "Pause"
	pause.toggled.connect(func(on: bool) -> void: _sim.set_paused(on))
	row.add_child(pause)

	var speed := HSlider.new()
	speed.min_value = 0.1
	speed.max_value = 4.0
	speed.step = 0.1
	speed.value = 1.0
	speed.custom_minimum_size = Vector2(120, 0)
	speed.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	speed.tooltip_text = "Sim speed"
	speed.value_changed.connect(func(value: float) -> void: _sim.set_speed(value))
	row.add_child(speed)

	var graphs_btn := Button.new()
	graphs_btn.toggle_mode = true
	graphs_btn.text = "Graphs"
	graphs_btn.toggled.connect(func(on: bool) -> void: _graphs.visible = on)
	row.add_child(graphs_btn)

	_stats_label = Label.new()
	_stats_label.add_theme_font_size_override("font_size", 12)
	col.add_child(_stats_label)

	_graphs = VBoxContainer.new()
	_graphs.visible = false
	col.add_child(_graphs)
	_graph_fps = PerfGraph.new("fps", "", Color(0.4, 1.0, 0.4))
	_graph_frame = PerfGraph.new("frame", " ms", Color(0.4, 0.7, 1.0))
	_graph_step = PerfGraph.new("sim step", " ms", Color(1.0, 0.7, 0.3))
	for graph in [_graph_fps, _graph_frame, _graph_step]:
		_graphs.add_child(graph)
