extends Control

@onready var delaunay: DelaunayTriangulator = DelaunayTriangulator.new()
@onready var polygon_2d: EdgeDrawPolygon2D = $"../../EdgeDrawPolygon2D"
@onready var navigation_region_2d: NavigationRegion2D = $"../../NavigationRegion2D"
var arr: PackedVector2Array = []

func _ready() -> void:
	set_physics_process(false)
	fill_arr_random(5000)

func _on_gui_input(event: InputEvent) -> void:
	if event is InputEventMouseButton and event.is_pressed():
		if event.button_index == 2: # Left mouse button
			add_constrained()
		arr.append(event.position)
		add_points_to_polygon()


func fill_arr_random(arr_size: int) -> void:
	for i in arr_size:
		arr.append(Vector2(randi() % get_window().size.x, randi() % get_window().size.y))
	add_points_to_polygon()


func add_points_to_polygon() -> void:
	if arr.size() < 3:
		return

	delaunay.set_points(arr)
	delaunay.triangulate()
	var triangles: Array[PackedInt32Array] = delaunay.get_indices()
	polygon_2d.polygon = arr
	polygon_2d.polygons = triangles


var cum_delta: float = 0.0
func _physics_process(delta: float) -> void:
	cum_delta += delta
	if cum_delta < 0.2:
		return

	arr.append(Vector2(randi() % get_window().size.x, randi() % get_window().size.y))
	print(arr.size())
	add_points_to_polygon()
	cum_delta = 0.0


var last_constrained_point: int = -1
var constrained_points: PackedInt32Array = []
func add_constrained() -> void:
	if last_constrained_point < 0:
		last_constrained_point = arr.size()
		return

	constrained_points.append(last_constrained_point)
	constrained_points.append(arr.size())
	last_constrained_point = -1
	delaunay.set_constraints(constrained_points)
