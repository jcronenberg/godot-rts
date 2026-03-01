extends Control

@onready var _editor: DelaunayEditorNode = $"../../DelaunayEditorNode"

var _start: Vector2
var _has_start: bool = false
var _path: PackedVector2Array = []

func _gui_input(event: InputEvent) -> void:
	if not (event is InputEventMouseButton and event.is_pressed()):
		return

	match event.button_index:
		MOUSE_BUTTON_LEFT:
			_start = event.position
			_has_start = true
			_path = []
			queue_redraw()

		MOUSE_BUTTON_RIGHT:
			if not _has_start:
				push_error("Left-click first to set a start point.")
				return
			var goal: Vector2 = event.position
			_path = _editor._triangulator.find_path(_start, goal)
			if _path.is_empty():
				print("No path from ", _start, " to ", goal)
			queue_redraw()

func _draw() -> void:
	if _has_start:
		draw_circle(_start, 6.0, Color.GREEN)

	if _path.size() < 2:
		return

	draw_polyline(_path, Color.YELLOW, 2.0)
	draw_circle(_path[-1], 6.0, Color.RED)
