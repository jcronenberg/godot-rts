extends Control

@onready var _editor: DelaunayEditorNode = $"../../DelaunayEditorNode"

@export var actor_size: float = 5.0

var _start: Vector2
var _has_start: bool = false
var _path: PackedVector2Array = []

func _process(_delta: float) -> void:
	# Start/path are stored in world space, but we draw in this Control's screen
	# space, so redraw while active to keep the overlay aligned as the camera moves.
	if _has_start:
		queue_redraw()

func _gui_input(event: InputEvent) -> void:
	if not (event is InputEventMouseButton and event.is_pressed()):
		return

	var world: Vector2 = _editor.get_global_transform_with_canvas().affine_inverse() * event.position

	match event.button_index:
		MOUSE_BUTTON_LEFT:
			_start = world
			_has_start = true
			_path = []
			queue_redraw()

		MOUSE_BUTTON_RIGHT:
			if not _has_start:
				push_error("Left-click first to set a start point.")
				return
			_path = _editor._triangulator.find_path(_start, world, actor_size)
			queue_redraw()

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
