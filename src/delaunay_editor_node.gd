@tool
class_name DelaunayEditorNode
extends Node2D

@export_group("Colors")
@export var floor_color: Color = Color(0.0, 0.5, 1, 0.25)
@export var edge_color: Color = Color(0.0, 0.0, 1.0)
@export var constraint_color: Color = Color(1.0, 0.0, 0.0)
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

const SNAP_RADIUS: float = 10.0
const POINT_RADIUS: float = 2.0


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
