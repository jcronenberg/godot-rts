extends TestSuite

func get_cases() -> Array:
	return [
		["test_3_points_gives_1_triangle", test_3_points_gives_1_triangle],
		["test_4_points_square_gives_2_triangles", test_4_points_square_gives_2_triangles],
		["test_triangle_count_matches_indices_size", test_triangle_count_matches_indices_size],
		["test_mesh_vertices_count_is_triangle_count_times_3", test_mesh_vertices_count_is_triangle_count_times_3],
		["test_indices_have_3_elements_each", test_indices_have_3_elements_each],
		["test_indices_in_valid_range", test_indices_in_valid_range],
		["test_with_constraint", test_with_constraint],
		["test_large_point_set", test_large_point_set],
		["test_retriangulate", test_retriangulate],
	]


func test_3_points_gives_1_triangle() -> Array[String]:
	var dt := DelaunayTriangulator.new()
	dt.set_points(PackedVector2Array([
		Vector2(0.0, 0.0),
		Vector2(1.0, 0.0),
		Vector2(0.5, 1.0),
	]))
	dt.triangulate()
	var f: Array[String] = []
	if dt.get_triangle_count() != 1:
		f.append("Expected 1 triangle, got %d" % dt.get_triangle_count())
	return f


func test_4_points_square_gives_2_triangles() -> Array[String]:
	var dt := DelaunayTriangulator.new()
	dt.set_points(PackedVector2Array([
		Vector2(0.0, 0.0),
		Vector2(1.0, 0.0),
		Vector2(1.0, 1.0),
		Vector2(0.0, 1.0),
	]))
	dt.triangulate()
	var f: Array[String] = []
	if dt.get_triangle_count() != 2:
		f.append("Expected 2 triangles, got %d" % dt.get_triangle_count())
	return f


func test_triangle_count_matches_indices_size() -> Array[String]:
	var dt := DelaunayTriangulator.new()
	dt.set_points(PackedVector2Array([
		Vector2(0.0, 0.0),
		Vector2(4.0, 0.0),
		Vector2(4.0, 4.0),
		Vector2(0.0, 4.0),
		Vector2(1.0, 1.0),
		Vector2(3.0, 1.0),
	]))
	dt.triangulate()
	var f: Array[String] = []
	var count := dt.get_triangle_count()
	var indices: Array[PackedInt32Array] = dt.get_indices()
	if indices.size() != count:
		f.append("get_indices().size() == %d, expected %d" % [indices.size(), count])
	return f


func test_mesh_vertices_count_is_triangle_count_times_3() -> Array[String]:
	var dt := DelaunayTriangulator.new()
	dt.set_points(PackedVector2Array([
		Vector2(0.0, 0.0),
		Vector2(2.0, 0.0),
		Vector2(2.0, 2.0),
		Vector2(0.0, 2.0),
		Vector2(1.0, 1.0),
	]))
	dt.triangulate()
	var f: Array[String] = []
	var count := dt.get_triangle_count()
	var verts := dt.get_mesh_vertices()
	if verts.size() != count * 3:
		f.append("get_mesh_vertices().size() == %d, expected %d" % [verts.size(), count * 3])
	return f


func test_indices_have_3_elements_each() -> Array[String]:
	var dt := DelaunayTriangulator.new()
	dt.set_points(PackedVector2Array([
		Vector2(0.0, 0.0),
		Vector2(2.0, 0.0),
		Vector2(2.0, 2.0),
		Vector2(0.0, 2.0),
		Vector2(1.0, 1.0),
	]))
	dt.triangulate()
	var f: Array[String] = []
	var indices: Array[PackedInt32Array] = dt.get_indices()
	for i in indices.size():
		if indices[i].size() != 3:
			f.append("Triangle %d has %d indices, expected 3" % [i, indices[i].size()])
	return f


func test_indices_in_valid_range() -> Array[String]:
	var pts := PackedVector2Array([
		Vector2(0.0, 0.0),
		Vector2(2.0, 0.0),
		Vector2(2.0, 2.0),
		Vector2(0.0, 2.0),
		Vector2(1.0, 1.0),
	])
	var dt := DelaunayTriangulator.new()
	dt.set_points(pts)
	dt.triangulate()
	var f: Array[String] = []
	var point_count := pts.size()
	var indices: Array[PackedInt32Array] = dt.get_indices()
	for i in indices.size():
		var tri := indices[i]
		for j in tri.size():
			var idx := tri[j]
			if idx < 0 or idx >= point_count:
				f.append("Triangle %d index %d = %d out of range [0, %d)" % [i, j, idx, point_count])
		if tri.size() == 3 and (tri[0] == tri[1] or tri[1] == tri[2] or tri[0] == tri[2]):
			f.append("Triangle %d has duplicate indices: [%d, %d, %d]" % [i, tri[0], tri[1], tri[2]])
	return f


func test_with_constraint() -> Array[String]:
	var pts := PackedVector2Array([
		Vector2(0.0, 0.0),
		Vector2(1.0, 0.0),
		Vector2(1.0, 1.0),
		Vector2(0.0, 1.0),
	])
	var dt := DelaunayTriangulator.new()
	dt.set_points(pts)
	dt.set_constraints(PackedInt32Array([0, 2]))
	dt.triangulate()
	var f: Array[String] = []

	if dt.get_triangle_count() != 2:
		f.append("Expected 2 triangles with constraint, got %d" % dt.get_triangle_count())

	# Check edge 0-2 is present in the triangulation
	var indices: Array[PackedInt32Array] = dt.get_indices()
	var edge_present := false
	for tri in indices:
		if tri.size() == 3:
			for k in 3:
				var a := tri[k]
				var b := tri[(k + 1) % 3]
				if (a == 0 and b == 2) or (a == 2 and b == 0):
					edge_present = true
					break
	if not edge_present:
		f.append("Constrained edge 0-2 not found in triangulation")

	# Consistency: indices.size() == triangle_count
	if indices.size() != dt.get_triangle_count():
		f.append("get_indices().size() == %d, expected %d" % [indices.size(), dt.get_triangle_count()])

	return f


func test_large_point_set() -> Array[String]:
	var pts := PackedVector2Array()
	var seed_val: int = 12345
	for _i in 500:
		seed_val = (seed_val * 1103515245 + 12345) & 0x7FFFFFFFFFFFFFFF
		var x := float(seed_val % 10000)
		seed_val = (seed_val * 1103515245 + 12345) & 0x7FFFFFFFFFFFFFFF
		var y := float(seed_val % 10000)
		pts.append(Vector2(x, y))

	var dt := DelaunayTriangulator.new()
	dt.set_points(pts)
	dt.triangulate()
	var f: Array[String] = []

	var count := dt.get_triangle_count()
	if count < 498:
		f.append("Expected >= 498 triangles for 500 points, got %d" % count)

	# Consistency checks
	var indices: Array[PackedInt32Array] = dt.get_indices()
	if indices.size() != count:
		f.append("get_indices().size() == %d, expected %d" % [indices.size(), count])

	var verts := dt.get_mesh_vertices()
	if verts.size() != count * 3:
		f.append("get_mesh_vertices().size() == %d, expected %d" % [verts.size(), count * 3])

	var point_count := pts.size()
	var out_of_range_found := false
	for i in indices.size():
		var tri := indices[i]
		for j in tri.size():
			var idx := tri[j]
			if idx < 0 or idx >= point_count:
				f.append("Triangle %d index %d = %d out of range [0, %d)" % [i, j, idx, point_count])
				out_of_range_found = true
				break
		if out_of_range_found:
			break

	return f


func test_retriangulate() -> Array[String]:
	var pts := PackedVector2Array([
		Vector2(0.0, 0.0),
		Vector2(2.0, 0.0),
		Vector2(2.0, 2.0),
		Vector2(0.0, 2.0),
		Vector2(1.0, 1.0),
	])
	var dt := DelaunayTriangulator.new()
	dt.set_points(pts)

	dt.triangulate()
	var count1 := dt.get_triangle_count()
	var indices1: Array[PackedInt32Array] = dt.get_indices()
	var verts1 := dt.get_mesh_vertices()

	dt.triangulate()
	var count2 := dt.get_triangle_count()
	var indices2: Array[PackedInt32Array] = dt.get_indices()
	var verts2 := dt.get_mesh_vertices()

	var f: Array[String] = []

	if count1 != count2:
		f.append("Triangle count differs between calls: %d vs %d" % [count1, count2])

	if indices1.size() != indices2.size():
		f.append("Indices size differs: %d vs %d" % [indices1.size(), indices2.size()])
	else:
		for i in indices1.size():
			for j in 3:
				if indices1[i][j] != indices2[i][j]:
					f.append("Index mismatch at triangle %d element %d: %d vs %d" % [i, j, indices1[i][j], indices2[i][j]])
					break

	if verts1.size() != verts2.size():
		f.append("Mesh vertices size differs: %d vs %d" % [verts1.size(), verts2.size()])
	else:
		for i in verts1.size():
			if verts1[i] != verts2[i]:
				f.append("Vertex mismatch at %d: %s vs %s" % [i, verts1[i], verts2[i]])
				break

	return f
