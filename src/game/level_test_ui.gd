extends Control

const ACTOR_SIZE_MIN: float = 1.0
const ACTOR_SIZE_MAX: float = 50.0
const OBSTACLE_SIZE_MIN: float = 1.0
const OBSTACLE_SIZE_MAX: float = 200.0

@export var actor_size: float = 5.0
@export var obstacle_size: Vector2 = Vector2(10, 10)

var _start: Vector2
var _has_start: bool = false
var _goal: Vector2
var _has_goal: bool = false
var _path: PackedVector2Array = []
var _actor_size_label: Label
var _obstacle_button: Button

@onready var _editor: DelaunayEditorNode = $"../../DelaunayEditorNode"
@onready var _buildings: BuildingOverlay = $"../../BuildingOverlay"


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
			if _obstacle_button.button_pressed:
				_insert_obstacle(world)
				return
			_start = world
			_has_start = true
			_has_goal = false
			_path = []
			queue_redraw()

		MOUSE_BUTTON_RIGHT:
			if _obstacle_button.button_pressed:
				_remove_obstacle_at(world)
				return
			if not _has_start:
				push_error("Left-click first to set a start point.")
				return
			_goal = world
			_has_goal = true
			_recompute_path()


func _insert_obstacle(center: Vector2) -> void:
	var half := obstacle_size / 2.0
	var box := PackedVector2Array(
		[
			center + Vector2(-half.x, -half.y),
			center + Vector2(half.x, -half.y),
			center + Vector2(half.x, half.y),
			center + Vector2(-half.x, half.y),
		]
	)
	var id := _editor.add_obstacle(box)
	if id == -1:
		return
	_buildings.add_building(id, Rect2(center - half, obstacle_size))
	_recompute_path()


func _remove_obstacle_at(pos: Vector2) -> void:
	var id := _buildings.remove_building_at(pos)
	if id == -1:
		return
	_editor.remove_obstacle(id)
	_recompute_path()


func _reset_map() -> void:
	var ids := _buildings.clear_buildings()
	if ids.is_empty():
		return
	for id in ids:
		_editor.remove_obstacle(id)
	_recompute_path()


func _recompute_path() -> void:
	if not (_has_start and _has_goal):
		return
	_path = _editor.find_path(_start, _goal, actor_size)
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

	_obstacle_button = Button.new()
	_obstacle_button.toggle_mode = true
	_obstacle_button.text = "Place obstacle"
	_obstacle_button.tooltip_text = "While on, left-click places an obstacle, right-click removes one"
	row.add_child(_obstacle_button)

	row.add_child(_make_size_spin_box(Vector2.AXIS_X))
	row.add_child(_make_size_spin_box(Vector2.AXIS_Y))

	var reset_button := Button.new()
	reset_button.text = "Reset map"
	reset_button.tooltip_text = "Remove all placed obstacles"
	reset_button.pressed.connect(_reset_map)
	row.add_child(reset_button)

	_update_actor_size_label()


func _make_size_spin_box(axis: int) -> SpinBox:
	var spin := SpinBox.new()
	spin.min_value = OBSTACLE_SIZE_MIN
	spin.max_value = OBSTACLE_SIZE_MAX
	spin.step = 0.5
	spin.value = obstacle_size[axis]
	spin.prefix = "x:" if axis == Vector2.AXIS_X else "y:"
	spin.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	spin.value_changed.connect(func(value: float) -> void: obstacle_size[axis] = value)
	return spin


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
