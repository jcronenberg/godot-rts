extends Control

const ACTOR_SIZE_MIN: float = 1.0
const ACTOR_SIZE_MAX: float = 50.0

@export var actor_size: float = 5.0

var _start: Vector2
var _has_start: bool = false
var _goal: Vector2
var _has_goal: bool = false
var _path: PackedVector2Array = []
var _actor_size_label: Label

@onready var _editor: DelaunayEditorNode = $"../../DelaunayEditorNode"


func _ready() -> void:
	_build_debug_ui()


func _process(_delta: float) -> void:
	# Start/path are stored in world space, but we draw in this Control's screen
	# space, so redraw while active to keep the overlay aligned as the camera moves.
	if _has_start:
		queue_redraw()


func _gui_input(event: InputEvent) -> void:
	if not (event is InputEventMouseButton and event.is_pressed()):
		return

	var world: Vector2 = (
		_editor.get_global_transform_with_canvas().affine_inverse() * event.position
	)

	match event.button_index:
		MOUSE_BUTTON_LEFT:
			_start = world
			_has_start = true
			_has_goal = false
			_path = []
			queue_redraw()

		MOUSE_BUTTON_RIGHT:
			if not _has_start:
				push_error("Left-click first to set a start point.")
				return
			_goal = world
			_has_goal = true
			_recompute_path()


func _recompute_path() -> void:
	if not (_has_start and _has_goal):
		return
	_path = _editor._triangulator.find_path(_start, _goal, actor_size)
	queue_redraw()


func _build_debug_ui() -> void:
	# Lives inside this full-screen Control; it only intercepts clicks on its own
	# rect, so pathfinding clicks elsewhere still reach _gui_input.
	var panel := PanelContainer.new()
	panel.position = Vector2(8, 8)
	add_child(panel)

	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 8)
	panel.add_child(row)

	_actor_size_label = Label.new()
	_actor_size_label.custom_minimum_size = Vector2(120, 0)
	row.add_child(_actor_size_label)

	var slider := HSlider.new()
	slider.min_value = ACTOR_SIZE_MIN
	slider.max_value = ACTOR_SIZE_MAX
	slider.step = 0.5
	slider.value = actor_size
	slider.custom_minimum_size = Vector2(180, 0)
	slider.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	slider.value_changed.connect(_on_actor_size_changed)
	row.add_child(slider)

	_update_actor_size_label()


func _on_actor_size_changed(value: float) -> void:
	actor_size = value
	_update_actor_size_label()
	_recompute_path()


func _update_actor_size_label() -> void:
	_actor_size_label.text = "Actor size: %.1f" % actor_size


func _draw() -> void:
	var to_screen := _editor.get_global_transform_with_canvas()

	if _has_start:
		draw_circle(to_screen * _start, 6.0, Color.GREEN)

	if _path.size() < 2:
		return

	var screen_path := PackedVector2Array()
	for point in _path:
		screen_path.append(to_screen * point)

	draw_polyline(screen_path, Color.YELLOW, 2.0)
	for point in screen_path:
		draw_circle(point, 1, Color.RED)
	draw_circle(screen_path[-1], 6.0, Color.RED)
