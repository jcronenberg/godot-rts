@tool
class_name DelaunayEditorNode
extends Node2D

@export_group("Colors")
@export var floor_color: Color = Color(0.0, 0.5, 1, 0.25)
@export var edge_color: Color = Color(0.0, 0.0, 1.0)
@export var constraint_color: Color = Color(1.0, 0.0, 0.0)
@export_group("")

@export_group("JSON")
@export_tool_button("Save to JSON") var _save_btn: Callable = _save_to_json
@export_tool_button("Load from JSON") var _load_btn: Callable = _load_from_json
@export_group("")

@export var points: PackedVector2Array = []:
	set(value):
		points = value
		triangulate()
@export var constraints: PackedInt32Array = []:
	set(value):
		constraints = value
		triangulate()

var _triangles: Array[PackedInt32Array] = []
var _triangulator: DelaunayTriangulator = DelaunayTriangulator.new()
var _pending_constraint_start: int = -1 # User adding constraint tracking
var _mouse_pos: Vector2 = Vector2.ZERO
var _save_dialog: EditorFileDialog = null
var _load_dialog: EditorFileDialog = null

const SNAP_RADIUS: float = 10.0
const POINT_RADIUS: float = 2.0


func _exit_tree() -> void:
	if _save_dialog:
		_save_dialog.queue_free()
		_save_dialog = null
	if _load_dialog:
		_load_dialog.queue_free()
		_load_dialog = null


func vp_to_local() -> Vector2:
	var vp_pos := EditorInterface.get_editor_viewport_2d().get_mouse_position()
	return get_global_transform().affine_inverse() * vp_pos


func find_nearest_point(pos: Vector2) -> int:
	var best_idx := -1
	var best_dist := SNAP_RADIUS
	for i in points.size():
		var d := pos.distance_to(points[i])
		if d < best_dist:
			best_dist = d
			best_idx = i
	return best_idx


func update_mouse_pos() -> void:
	_mouse_pos = vp_to_local()
	queue_redraw()


func handle_add_click() -> void:
	points.append(vp_to_local())
	queue_redraw()


func handle_constraint_click() -> void:
	var mouse_pos := vp_to_local()
	var nearest_idx := find_nearest_point(mouse_pos)
	if nearest_idx == -1:
		points.append(mouse_pos)
		nearest_idx = points.size() - 1

	if _pending_constraint_start == -1:
		_pending_constraint_start = nearest_idx
	else:
		var a := _pending_constraint_start
		var b := nearest_idx
		_pending_constraint_start = -1
		# Toggle: remove if already exists, add if not.
		for i in range(0, constraints.size() - 1, 2):
			if (constraints[i] == a and constraints[i + 1] == b) or \
			   (constraints[i] == b and constraints[i + 1] == a):
				constraints.remove_at(i + 1)
				constraints.remove_at(i)
				queue_redraw()
				return
		constraints.append(a)
		constraints.append(b)
		queue_redraw()


func move_point(idx: int, pos: Vector2) -> void:
	points[idx] = pos
	queue_redraw()


func erase_point(idx: int) -> void:
	points.remove_at(idx)

	var new_constraints := PackedInt32Array()
	for i in range(0, constraints.size() - 1, 2):
		var a := constraints[i]
		var b := constraints[i + 1]
		if a == idx or b == idx:
			continue
		new_constraints.append(a - int(a > idx))
		new_constraints.append(b - int(b > idx))
	constraints = new_constraints

	if _pending_constraint_start == idx:
		_pending_constraint_start = -1
	elif _pending_constraint_start > idx:
		_pending_constraint_start -= 1

	_triangles.clear()
	queue_redraw()


func triangulate() -> void:
	if points.size() < 3:
		_triangles.clear()
		queue_redraw()
		return
	_triangulator.set_points(points)
	_triangulator.set_constraints(constraints)
	_triangulator.triangulate()
	_triangles = _triangulator.get_indices()
	queue_redraw()


func clear_all() -> void:
	points.clear()
	constraints.clear()
	_triangles.clear()
	_pending_constraint_start = -1
	queue_redraw()


func _save_to_json() -> void:
	if _save_dialog == null:
		_save_dialog = EditorFileDialog.new()
		_save_dialog.file_mode = EditorFileDialog.FILE_MODE_SAVE_FILE
		_save_dialog.access = EditorFileDialog.ACCESS_FILESYSTEM
		_save_dialog.add_filter("*.json", "JSON Map Files")
		_save_dialog.file_selected.connect(_on_save_file_selected)
		EditorInterface.get_base_control().add_child(_save_dialog)
	_save_dialog.popup_centered_ratio(0.5)


func _load_from_json() -> void:
	if _load_dialog == null:
		_load_dialog = EditorFileDialog.new()
		_load_dialog.file_mode = EditorFileDialog.FILE_MODE_OPEN_FILE
		_load_dialog.access = EditorFileDialog.ACCESS_FILESYSTEM
		_load_dialog.add_filter("*.json", "JSON Map Files")
		_load_dialog.file_selected.connect(_on_load_file_selected)
		EditorInterface.get_base_control().add_child(_load_dialog)
	_load_dialog.popup_centered_ratio(0.5)


func _on_save_file_selected(path: String) -> void:
	var data := {
		"points": Array(points).map(func(p: Vector2) -> Array: return [p.x, p.y]),
		"constraints": Array(constraints),
	}
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file == null:
		push_error("DelaunayEditorNode: could not open file for writing: " + path)
		return
	file.store_string(JSON.stringify(data, "\t"))
	file.close()


func _on_load_file_selected(path: String) -> void:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		push_error("DelaunayEditorNode: could not open file for reading: " + path)
		return
	var data = JSON.parse_string(file.get_as_text())
	file.close()
	if data == null:
		push_error("DelaunayEditorNode: failed to parse JSON: " + path)
		return
	var new_points := PackedVector2Array()
	for p in data.get("points", []):
		new_points.append(Vector2(float(p[0]), float(p[1])))
	var new_constraints := PackedInt32Array()
	for c in data.get("constraints", []):
		new_constraints.append(int(c))
	points = new_points
	constraints = new_constraints
	notify_property_list_changed()


func _draw() -> void:
	# Floor fill
	for tri in _triangles:
		var verts := PackedVector2Array([points[tri[0]], points[tri[1]], points[tri[2]]])
		draw_polygon(verts, PackedColorArray([floor_color]))

	# Edges
	for tri in _triangles:
		draw_line(points[tri[0]], points[tri[1]], edge_color, 1.0)
		draw_line(points[tri[1]], points[tri[2]], edge_color, 1.0)
		draw_line(points[tri[2]], points[tri[0]], edge_color, 1.0)

	# Constraints
	for i in range(0, constraints.size() - 1, 2):
		draw_line(points[constraints[i]], points[constraints[i + 1]], constraint_color, 2.0)

	# Pending constraint preview
	if _pending_constraint_start >= 0:
		draw_line(points[_pending_constraint_start], _mouse_pos, constraint_color, 1.0)

	for p in points:
		draw_circle(p, POINT_RADIUS, Color.WHITE)
