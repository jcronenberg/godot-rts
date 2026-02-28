@tool
extends EditorPlugin

enum Mode { ADD, CONSTRAINT, EDIT, ERASE }

var _active_node: DelaunayEditorNode = null
var _toolbar: HBoxContainer = null
var _mode: Mode = Mode.ADD
var _drag_idx: int = -1


func _make_transparent(btn: Button) -> void:
	btn.flat = true


func _enter_tree() -> void:
	_toolbar = HBoxContainer.new()

	var group := ButtonGroup.new()
	for entry in [["Add", Mode.ADD], ["Constraint", Mode.CONSTRAINT], ["Edit", Mode.EDIT], ["Erase", Mode.ERASE]]:
		var btn := Button.new()
		btn.text = entry[0]
		btn.toggle_mode = true
		btn.button_group = group
		btn.pressed.connect(_set_mode.bind(entry[1]))
		_make_transparent(btn)
		var accent := EditorInterface.get_editor_theme().get_color("accent_color", "Editor")
		btn.add_theme_color_override("font_pressed_color", accent)
		btn.add_theme_color_override("font_hover_pressed_color", accent)
		_toolbar.add_child(btn)

	# Start with Add selected.
	(_toolbar.get_child(0) as Button).button_pressed = true

	var sep := VSeparator.new()
	_toolbar.add_child(sep)

	var clear_btn := Button.new()
	clear_btn.text = "Clear All"
	clear_btn.pressed.connect(func(): _active_node.clear_all(); _active_node.notify_property_list_changed())
	_make_transparent(clear_btn)
	_toolbar.add_child(clear_btn)

	add_control_to_container(CONTAINER_CANVAS_EDITOR_MENU, _toolbar)
	_toolbar.hide()

	EditorInterface.get_selection().selection_changed.connect(_on_selection_changed)
	_on_selection_changed()


func _exit_tree() -> void:
	EditorInterface.get_selection().selection_changed.disconnect(_on_selection_changed)
	remove_control_from_container(CONTAINER_CANVAS_EDITOR_MENU, _toolbar)
	_toolbar.queue_free()
	_toolbar = null
	_active_node = null


func _set_mode(mode: Mode) -> void:
	_mode = mode
	_drag_idx = -1


func _auto_triangulate() -> void:
	_active_node.triangulate()
	_active_node.notify_property_list_changed()


func _on_selection_changed() -> void:
	_drag_idx = -1
	if _active_node:
		_active_node._pending_constraint_start = -1
		_active_node.queue_redraw()
	_active_node = null
	for node in EditorInterface.get_selection().get_selected_nodes():
		if node is DelaunayEditorNode:
			_active_node = node
			break

	if _active_node:
		_toolbar.show()
	else:
		_toolbar.hide()

	update_overlays()


func _forward_canvas_gui_input(event: InputEvent) -> bool:
	if _active_node == null:
		return false

	match _mode:
		Mode.ADD:
			return _handle_add(event)
		Mode.CONSTRAINT:
			return _handle_constraint(event)
		Mode.EDIT:
			return _handle_edit(event)
		Mode.ERASE:
			return _handle_erase(event)

	return false


func _handle_add(event: InputEvent) -> bool:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
		_active_node.handle_add_click()
		_auto_triangulate()
		return true

	return false


func _handle_constraint(event: InputEvent) -> bool:
	if event is InputEventMouseMotion:
		_active_node.update_mouse_pos()
		update_overlays()
		return false

	if event is InputEventMouseButton and event.pressed:
		if event.button_index == MOUSE_BUTTON_LEFT:
			_active_node.handle_constraint_click()
			_auto_triangulate()
			return true
		if event.button_index == MOUSE_BUTTON_RIGHT and _active_node._pending_constraint_start >= 0:
			_active_node._pending_constraint_start = -1
			_active_node.queue_redraw()
			return true

	return false


func _handle_edit(event: InputEvent) -> bool:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT:
		if event.pressed:
			var pos := _active_node.vp_to_local()
			_drag_idx = _active_node.find_nearest_point(pos)
			return _drag_idx >= 0
		else:
			var was_dragging := _drag_idx >= 0
			_drag_idx = -1
			if was_dragging:
				_auto_triangulate()
			return was_dragging

	if event is InputEventMouseMotion and _drag_idx >= 0:
		_active_node.move_point(_drag_idx, _active_node.vp_to_local())
		return true  # consume motion while dragging to suppress camera pan

	return false


func _handle_erase(event: InputEvent) -> bool:
	if event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and event.pressed:
		var pos := _active_node.vp_to_local()
		var idx := _active_node.find_nearest_point(pos)
		if idx >= 0:
			_active_node.erase_point(idx)
			_auto_triangulate()
			return true

	return false


func _handles(object: Object) -> bool:
	return true
