class_name RtsCamera
extends Camera2D

## Keyboard / edge-scroll pan speed in pixels per second (measured on screen,
## so it feels the same at every zoom level).
@export var pan_speed: float = 800.0
## Distance in pixels from a viewport edge that triggers edge scrolling.
@export var edge_margin: float = 16.0
## Move the camera by pushing the mouse against the screen edges.
@export var edge_pan_enabled: bool = true
## Pan by holding the middle mouse button and dragging.
@export var drag_pan_enabled: bool = true
## Zoom multiplier applied per mouse-wheel notch.
@export var zoom_factor: float = 1.1
@export var min_zoom: float = 0.4
@export var max_zoom: float = 4.0

var _dragging: bool = false


func _process(delta: float) -> void:
	var dir := _keyboard_dir()
	if edge_pan_enabled and not _dragging:
		dir += _edge_dir()
	if dir != Vector2.ZERO:
		# Divide by zoom so the on-screen pan speed stays constant when zoomed.
		global_position += dir.normalized() * pan_speed * delta / zoom.x


# Handled in _input (which runs before GUI) so the full-screen UI Control does
# not swallow the wheel and middle-button events.
func _input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.is_pressed():
		match event.button_index:
			MOUSE_BUTTON_WHEEL_UP:
				_apply_zoom(zoom_factor)
				get_viewport().set_input_as_handled()
			MOUSE_BUTTON_WHEEL_DOWN:
				_apply_zoom(1.0 / zoom_factor)
				get_viewport().set_input_as_handled()

	if (
		drag_pan_enabled
		and event is InputEventMouseButton
		and event.button_index == MOUSE_BUTTON_MIDDLE
	):
		_dragging = event.is_pressed()
		get_viewport().set_input_as_handled()

	if _dragging and event is InputEventMouseMotion:
		global_position -= event.relative / zoom.x
		get_viewport().set_input_as_handled()


func _keyboard_dir() -> Vector2:
	return Input.get_vector("pan_left", "pan_right", "pan_up", "pan_down")


func _edge_dir() -> Vector2:
	if not get_window().has_focus():
		return Vector2.ZERO
	var size := get_viewport().get_visible_rect().size
	var mouse := get_viewport().get_mouse_position()
	# Ignore the mouse while it sits outside the viewport.
	if mouse.x < 0.0 or mouse.y < 0.0 or mouse.x > size.x or mouse.y > size.y:
		return Vector2.ZERO
	var dir := Vector2.ZERO
	if mouse.x < edge_margin:
		dir.x -= 1.0
	elif mouse.x > size.x - edge_margin:
		dir.x += 1.0
	if mouse.y < edge_margin:
		dir.y -= 1.0
	elif mouse.y > size.y - edge_margin:
		dir.y += 1.0
	return dir


func _apply_zoom(factor: float) -> void:
	# Anchor is centered, so just setting zoom keeps the screen center fixed.
	var new_zoom := clampf(zoom.x * factor, min_zoom, max_zoom)
	zoom = Vector2(new_zoom, new_zoom)
