@tool
class_name DelaunayEditorNode
extends Node2D

const SNAP_RADIUS: float = 10.0
const POINT_RADIUS: float = 2.0

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

var _triangle_soup: PackedVector2Array = []  # 3 vertices per face
var _constrained_edges: PackedVector2Array = []  # Segment soup, 2 vertices per edge
var _mesh_vertices: PackedVector2Array = []  # Unique vertices, including inserted obstacle ones
var _triangulator: DelaunayTriangulator = DelaunayTriangulator.new()
var _pending_constraint_start: int = -1  # User adding constraint tracking
var _mouse_pos: Vector2 = Vector2.ZERO
var _save_dialog: EditorFileDialog = null
var _load_dialog: EditorFileDialog = null
var _top_layer: TopLayer = null


# Constrained edges and points draw at z 2 so game overlays (e.g. buildings
# at z 1) can slot between them and the floor/mesh at z 0.
class TopLayer:
	extends Node2D
	var editor: DelaunayEditorNode

	func _draw() -> void:
		editor._draw_top(self)


func _ready() -> void:
	_top_layer = TopLayer.new()
	_top_layer.editor = self
	_top_layer.z_index = 2
	add_child(_top_layer, false, Node.INTERNAL_MODE_BACK)


func _request_redraw() -> void:
	queue_redraw()
	if _top_layer:
		_top_layer.queue_redraw()


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
	_request_redraw()


func handle_add_click() -> void:
	points.append(vp_to_local())
	_request_redraw()


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
			if (
				(constraints[i] == a and constraints[i + 1] == b)
				or (constraints[i] == b and constraints[i + 1] == a)
			):
				constraints.remove_at(i + 1)
				constraints.remove_at(i)
				_request_redraw()
				return
		constraints.append(a)
		constraints.append(b)
		_request_redraw()


func move_point(idx: int, pos: Vector2) -> void:
	points[idx] = pos
	_request_redraw()


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

	_clear_mesh()
	_request_redraw()


func triangulate() -> void:
	if points.size() < 3:
		_clear_mesh()
		_request_redraw()
		return
	_triangulator.set_points(points)
	_triangulator.set_constraints(constraints)
	_triangulator.triangulate()
	refresh_mesh()


func refresh_mesh() -> void:
	_triangle_soup = _triangulator.get_mesh_vertices()
	_constrained_edges = _triangulator.get_constrained_edges()
	_mesh_vertices = _triangulator.get_points()
	_request_redraw()


func _clear_mesh() -> void:
	_triangle_soup.clear()
	_constrained_edges.clear()
	_mesh_vertices.clear()


## Insert a closed-polygon obstacle; returns its id (-1 on error).
func add_obstacle(polygon: PackedVector2Array) -> int:
	var id: int = _triangulator.add_obstacle(polygon)
	if id != -1:
		refresh_mesh()
	return id


func remove_obstacle(id: int) -> void:
	_triangulator.remove_obstacle(id)
	refresh_mesh()


func find_path(start: Vector2, goal: Vector2, radius: float) -> PackedVector2Array:
	return _triangulator.find_path(start, goal, radius)


## Display an externally produced mesh (e.g. the sim thread's live navmesh
## dump) instead of this node's own triangulation. `indices` hold 3 vertex
## indices per face; `edges` are constrained-edge endpoint pairs.
func show_mesh_dump(
	vertices: PackedVector2Array, indices: PackedInt32Array, edges: PackedVector2Array
) -> void:
	var soup := PackedVector2Array()
	soup.resize(indices.size())
	for i in indices.size():
		soup[i] = vertices[indices[i]]
	_triangle_soup = soup
	_constrained_edges = edges
	_mesh_vertices = vertices
	_request_redraw()


func clear_all() -> void:
	points.clear()
	constraints.clear()
	_clear_mesh()
	_pending_constraint_start = -1
	_request_redraw()


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
	for base in range(0, _triangle_soup.size(), 3):
		draw_polygon(_triangle_soup.slice(base, base + 3), PackedColorArray([floor_color]))

	# Edges
	for base in range(0, _triangle_soup.size(), 3):
		draw_line(_triangle_soup[base], _triangle_soup[base + 1], edge_color, 1.0)
		draw_line(_triangle_soup[base + 1], _triangle_soup[base + 2], edge_color, 1.0)
		draw_line(_triangle_soup[base + 2], _triangle_soup[base], edge_color, 1.0)


func _draw_top(ci: CanvasItem) -> void:
	# Constrained edges (map constraints plus inserted obstacles)
	for base in range(0, _constrained_edges.size(), 2):
		ci.draw_line(_constrained_edges[base], _constrained_edges[base + 1], constraint_color, 2.0)

	# Pending constraint preview
	if _pending_constraint_start >= 0:
		ci.draw_line(points[_pending_constraint_start], _mouse_pos, constraint_color, 1.0)

	# Mesh vertices when triangulated (includes obstacle vertices), raw points otherwise
	var markers := _mesh_vertices if not _mesh_vertices.is_empty() else points
	for p in markers:
		ci.draw_circle(p, POINT_RADIUS, Color.WHITE)
