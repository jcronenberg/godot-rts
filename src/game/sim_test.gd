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
var _order_goals: Array[PackedVector2Array] = []

var _stats_label: Label
var _graph_fps: PerfGraph
var _graph_frame: PerfGraph
var _graph_step: PerfGraph
var _stats_timer: float = 0.0
var _frame_ms_acc: float = 0.0
var _frame_count: int = 0
var _last_stat_tick: int = -1

var _dragged_panel: Control = null
var _drag_panel_offset: Vector2

# Panel references for initial edge positioning
var _keymap_panel: Control
var _controls_panel: Control
var _tuning_panel: Control


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

	var layer := CanvasLayer.new()
	add_child(layer)
	_build_perf_ui(layer)
	_build_keymap_ui(layer)
	_build_controls_ui(layer)
	_build_tuning_ui(layer)

	# Defer bottom-edge positioning until after first layout pass.
	_reposition_bottom_panels.call_deferred()


func _reposition_bottom_panels() -> void:
	await get_tree().process_frame
	var vp := get_viewport_rect().size
	_keymap_panel.position = Vector2(8.0, vp.y - _keymap_panel.size.y - 8.0)
	_controls_panel.position = Vector2(vp.x - _controls_panel.size.x - 8.0,
			vp.y - _controls_panel.size.y - 8.0)
	_tuning_panel.position = Vector2(vp.x - _tuning_panel.size.x - 8.0, 8.0)


func _process(delta: float) -> void:
	if _dragged_panel:
		_dragged_panel.position = get_viewport().get_mouse_position() + _drag_panel_offset
	_alpha = _sim.poll(delta)
	_ids = _sim.get_unit_ids()
	_positions = _sim.get_positions(_alpha)
	_radii = _sim.get_radii()
	_prune_selection()
	if _overlay:
		_refresh_overlay()
	_update_stats(delta)
	queue_redraw()


func _input(event: InputEvent) -> void:
	if _dragged_panel and event is InputEventMouseButton and not event.pressed:
		_dragged_panel = null


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
	_order_goals.assign(_sim.get_unit_order_goals())


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
					_move_selected(world, event.shift_pressed)
	elif event is InputEventMouseMotion and _dragging:
		_drag_end = get_global_mouse_position()
	elif event is InputEventKey and event.is_pressed() and event.keycode == KEY_U:
		var count := 50 if event.ctrl_pressed else (10 if event.shift_pressed else 1)
		var center := get_global_mouse_position()
		var spacing := unit_radius * 2.0 + 2.0
		var pack_r := spacing * sqrt(count / PI)
		var golden_angle := PI * (3.0 - sqrt(5.0))
		for i in count:
			var r := sqrt(float(i + 0.5) / count) * pack_r
			var a := i * golden_angle
			_sim.spawn_unit(center + Vector2(cos(a), sin(a)) * r, unit_radius, unit_speed)


func _select(world: Vector2) -> void:
	_selected.clear()
	if _drag_start.distance_to(_drag_end) < DRAG_THRESHOLD:
		# Plain click: select the first unit under the cursor.
		for i in _ids.size():
			if world.distance_to(_positions[i]) <= _radii[i] + 2.0:
				_selected[_ids[i]] = true
				return
		return
	var rect := Rect2(_drag_start, Vector2.ZERO).expand(_drag_end)
	for i in _ids.size():
		if rect.has_point(_positions[i]):
			_selected[_ids[i]] = true


func _move_selected(goal: Vector2, queued: bool) -> void:
	var ids := PackedInt64Array()
	for id in _selected.keys():
		ids.append(id)
	if queued:
		_sim.queue_move(ids, goal)
	else:
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
		_draw_queued_orders()

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


## Faint cyan polyline + dots from each unit's current goal through its queued
## waypoints, so the order queue is visible while the overlay is on.
func _draw_queued_orders() -> void:
	for i in _order_goals.size():
		var goals := _order_goals[i]
		if goals.is_empty():
			continue
		var has_path := i < _paths.size() and _paths[i].size() >= 2
		var start: Vector2 = _paths[i][_paths[i].size() - 1] if has_path else _positions[i]
		var line := PackedVector2Array([start])
		line.append_array(goals)
		draw_polyline(line, Color(0.2, 0.9, 1.0, 0.5), 1.0)
		for g in goals:
			draw_circle(g, 3.0, Color(0.2, 0.9, 1.0, 0.7))


## Creates a draggable, collapsible floating panel. Returns [panel, content].
## pin_bottom/pin_right: keep that edge fixed when expanding/collapsing (use
## for panels anchored to that edge of the screen).
func _make_floating_panel(
	parent: Node,
	title_text: String,
	collapsed: bool = false,
	pin_bottom: bool = false,
	pin_right: bool = false,
) -> Array:
	var panel := PanelContainer.new()
	parent.add_child(panel)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 2)
	panel.add_child(vbox)

	var title_row := HBoxContainer.new()
	vbox.add_child(title_row)

	# Dragging is triggered by holding the title button.
	var drag_btn := Button.new()
	drag_btn.flat = true
	drag_btn.text = title_text
	drag_btn.alignment = HORIZONTAL_ALIGNMENT_LEFT
	drag_btn.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	drag_btn.button_down.connect(func() -> void:
		_dragged_panel = panel
		_drag_panel_offset = panel.position - get_viewport().get_mouse_position()
	)
	title_row.add_child(drag_btn)

	var collapse_btn := Button.new()
	collapse_btn.text = "+" if collapsed else "−"
	collapse_btn.toggle_mode = true
	collapse_btn.button_pressed = not collapsed
	collapse_btn.flat = true
	title_row.add_child(collapse_btn)

	var content := VBoxContainer.new()
	content.add_theme_constant_override("separation", 4)
	content.visible = not collapsed
	vbox.add_child(content)

	collapse_btn.toggled.connect(func(on: bool) -> void:
		var old_bottom := panel.position.y + panel.size.y
		var old_right := panel.position.x + panel.size.x
		content.visible = on
		collapse_btn.text = "−" if on else "+"
		panel.reset_size()
		if pin_bottom:
			panel.position.y = old_bottom - panel.size.y
		if pin_right:
			panel.position.x = old_right - panel.size.x
	)

	return [panel, content]


func _build_perf_ui(layer: CanvasLayer) -> void:
	var r := _make_floating_panel(layer, "Performance", false)
	var panel: Control = r[0]
	var content: VBoxContainer = r[1]
	panel.position = Vector2(8.0, 8.0)

	_stats_label = Label.new()
	_stats_label.add_theme_font_size_override("font_size", 12)
	content.add_child(_stats_label)

	var graphs := VBoxContainer.new()
	content.add_child(graphs)
	_graph_fps = PerfGraph.new("fps", "", Color(0.4, 1.0, 0.4))
	_graph_frame = PerfGraph.new("frame", " ms", Color(0.4, 0.7, 1.0))
	_graph_step = PerfGraph.new("sim step", " ms", Color(1.0, 0.7, 0.3))
	for graph in [_graph_fps, _graph_frame, _graph_step]:
		graphs.add_child(graph)


func _build_keymap_ui(layer: CanvasLayer) -> void:
	var r := _make_floating_panel(layer, "Keymap", true, true)
	_keymap_panel = r[0]
	var content: VBoxContainer = r[1]
	_keymap_panel.position = Vector2(8.0, 400.0)  # corrected in _reposition_bottom_panels

	var entries: Array[String] = [
		"U — spawn unit at cursor",
		"Shift+U — spawn ×10",
		"Ctrl+U — spawn ×50",
		"Drag (LMB) — box-select units",
		"Click (LMB) — select unit",
		"RMB — move selected",
		"Shift+RMB — queue move",
		"LMB (build mode) — place building",
		"RMB (build mode) — remove building",
	]
	for entry in entries:
		var lbl := Label.new()
		lbl.text = entry
		lbl.add_theme_font_size_override("font_size", 11)
		content.add_child(lbl)


func _build_controls_ui(layer: CanvasLayer) -> void:
	var r := _make_floating_panel(layer, "Controls", false, true)
	_controls_panel = r[0]
	var content: VBoxContainer = r[1]
	_controls_panel.position = Vector2(400.0, 400.0)  # corrected in _reposition_bottom_panels

	var build := Button.new()
	build.toggle_mode = true
	build.text = "Place building"
	build.tooltip_text = "LMB places a building, RMB removes one"
	build.toggled.connect(func(on: bool) -> void: _build_mode = on)
	content.add_child(build)

	var overlay := Button.new()
	overlay.toggle_mode = true
	overlay.text = "Debug overlay"
	overlay.toggled.connect(func(on: bool) -> void:
		_overlay = on
		_mesh_version = -1
		_paths = []
		_order_goals = []
		_editor.visible = on
		_sim.set_debug_overlay(on)
	)
	overlay.button_pressed = true
	content.add_child(overlay)

	var pause := Button.new()
	pause.toggle_mode = true
	pause.text = "Pause"
	pause.toggled.connect(func(on: bool) -> void: _sim.set_paused(on))
	content.add_child(pause)

	var speed_row := HBoxContainer.new()
	content.add_child(speed_row)
	var speed_lbl := Label.new()
	speed_lbl.text = "Speed"
	speed_lbl.custom_minimum_size = Vector2(40, 0)
	speed_row.add_child(speed_lbl)
	var speed := HSlider.new()
	speed.min_value = 0.1
	speed.max_value = 4.0
	speed.step = 0.1
	speed.value = 1.0
	speed.custom_minimum_size = Vector2(120, 0)
	speed.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	speed.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	speed.tooltip_text = "Sim speed multiplier"
	speed.value_changed.connect(func(value: float) -> void: _sim.set_speed(value))
	speed_row.add_child(speed)

	var sep := HSeparator.new()
	content.add_child(sep)

	var unit_title := Label.new()
	unit_title.text = "Unit Properties"
	content.add_child(unit_title)

	_add_prop_slider(content, "Radius", unit_radius, 2.0, 30.0, 0.5,
		func(v: float) -> void: unit_radius = v)
	_add_prop_slider(content, "Speed", unit_speed, 10.0, 300.0, 5.0,
		func(v: float) -> void: unit_speed = v)


## Sliders for the sim's runtime-tunable flocking/pathing constants
## (rust/rts-lib/src/sim.rs), applied live via Simulation.set_tuning.
func _build_tuning_ui(layer: CanvasLayer) -> void:
	var r := _make_floating_panel(layer, "Tuning", true, false, true)
	_tuning_panel = r[0]
	var content: VBoxContainer = r[1]
	_tuning_panel.position = Vector2(280.0, 8.0)  # corrected in _reposition_bottom_panels

	# name, min, max, step — current value comes from Simulation.get_tuning,
	# not a hardcoded copy of the sim.rs default.
	var tunables := [
		["separation_relax", 0.0, 1.0, 0.05],
		["separation_max_frac", 0.0, 3.0, 0.1],
		["cohesion_radius_frac", 0.0, 10.0, 0.5],
		["cohesion_gain", 0.0, 0.3, 0.01],
		["cohesion_max_frac", 0.0, 0.5, 0.01],
		["arrival_touch_frac", 1.0, 2.0, 0.05],
		["arrival_radius_factor", 0.0, 3.0, 0.1],
		["arrival_min_radii", 0.0, 10.0, 0.5],
		["fan_frac", 0.0, 1.0, 0.02],
		["straight_fan_frac", 0.0, 1.0, 0.02],
		["stall_repath_ticks", 0.0, 20.0, 1.0],
		["stall_progress_eps", 0.0, 1.0, 0.05],
	]
	for t in tunables:
		var tuning_name: String = t[0]
		_add_prop_slider(content, tuning_name, _sim.get_tuning(tuning_name), t[1], t[2], t[3],
			func(v: float) -> void: _sim.set_tuning(tuning_name, v))


func _add_prop_slider(
	parent: Control,
	label: String,
	initial: float,
	min_val: float,
	max_val: float,
	step: float,
	on_change: Callable,
) -> void:
	var row := HBoxContainer.new()
	parent.add_child(row)

	var lbl := Label.new()
	lbl.text = label
	lbl.custom_minimum_size = Vector2(52, 0)
	row.add_child(lbl)

	var val_lbl := Label.new()
	val_lbl.text = "%.1f" % initial
	val_lbl.custom_minimum_size = Vector2(36, 0)
	row.add_child(val_lbl)

	var slider := HSlider.new()
	slider.min_value = min_val
	slider.max_value = max_val
	slider.step = step
	slider.value = initial
	slider.custom_minimum_size = Vector2(120, 0)
	slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	slider.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	row.add_child(slider)

	slider.value_changed.connect(func(v: float) -> void:
		val_lbl.text = "%.1f" % v
		on_change.call(v)
	)
