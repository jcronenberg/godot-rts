extends MainLoop
# Headless benchmarks for the DelaunayTriangulator GDExtension API.
# Datasets mirror rust/rts-lib/benches (same LCG, seeds, constraint pattern,
# rooms map), so subtracting the criterion numbers isolates FFI/marshaling cost.
# Run via benches/run_benches.sh (stages the release build for the editor binary).

const PAIRS := 256

var _lcg_state: int = 0


func _initialize() -> void:
	print("=== RTS GDExtension Benchmarks ===")
	print("(times are us/op; batched benches report per-op time)\n")

	_bench_pipeline_random(5000, 0x10AD5EED, 15, 3)
	_bench_pipeline_random(10000, 0x10AD5EED, 10, 2)
	_bench_pipeline_rooms(16, 15, 3)
	_bench_paths_random(10000, 0x12345678)
	_bench_paths_rooms(16)
	_bench_marshal(10000, 0x12345678)

	print("\ndone")


func _process(_delta: float) -> bool:
	return true


# ── benchmarks ────────────────────────────────────────────────────────────────


# Mirrors criterion pipeline/load: triangulate + constraints + abstraction.
func _bench_pipeline_random(n: int, point_seed: int, iters: int, warmup: int) -> void:
	var points := _random_points(n, point_seed)
	var cons := _chain_constraints(n)
	var dt := DelaunayTriangulator.new()
	var run := func() -> void:
		dt.set_points(points)
		dt.set_constraints(cons)
		dt.triangulate()
		dt.build_abstraction()
	_bench("pipeline/load/random/%d" % n, iters, warmup, 1, run)


# Mirrors criterion pipeline/load_rooms.
func _bench_pipeline_rooms(side: int, iters: int, warmup: int) -> void:
	var m := _rooms_map(side, side)
	var points: PackedVector2Array = m[0]
	var cons: PackedInt32Array = m[1]
	var dt := DelaunayTriangulator.new()
	var run := func() -> void:
		dt.set_points(points)
		dt.set_constraints(cons)
		dt.triangulate()
		dt.build_abstraction()
	_bench("pipeline/load_rooms/%d" % (side * side), iters, warmup, 1, run)


func _bench_paths_random(n: int, point_seed: int) -> void:
	var dt := DelaunayTriangulator.new()
	dt.set_points(_random_points(n, point_seed))
	dt.set_constraints(_chain_constraints(n))
	dt.triangulate()
	dt.build_abstraction()
	_bench_path_pairs(dt, "random/%d" % n)


func _bench_paths_rooms(side: int) -> void:
	var m := _rooms_map(side, side)
	var dt := DelaunayTriangulator.new()
	dt.set_points(m[0])
	dt.set_constraints(m[1])
	dt.triangulate()
	dt.build_abstraction()
	_bench_path_pairs(dt, "rooms/%d" % (side * side))


func _bench_path_pairs(dt: DelaunayTriangulator, label: String) -> void:
	var pairs := _make_pairs(dt, PAIRS)
	var starts: PackedVector2Array = pairs[0]
	var goals: PackedVector2Array = pairs[1]
	var run_astar := func() -> void:
		for i in PAIRS:
			dt.find_path(starts[i], goals[i], 0.0)
	var run_tra := func() -> void:
		for i in PAIRS:
			dt.find_path_tra(starts[i], goals[i], 0.0)
	_bench("find_path/%s" % label, 15, 2, PAIRS, run_astar)
	_bench("find_path_tra/%s" % label, 15, 2, PAIRS, run_tra)


func _bench_marshal(n: int, point_seed: int) -> void:
	var dt := DelaunayTriangulator.new()
	dt.set_points(_random_points(n, point_seed))
	dt.set_constraints(_chain_constraints(n))
	dt.triangulate()

	var run_verts := func() -> void: dt.get_mesh_vertices()
	var run_indices := func() -> void: dt.get_indices()
	_bench("marshal/get_mesh_vertices/%d" % n, 30, 3, 1, run_verts)
	_bench("marshal/get_indices/%d" % n, 20, 2, 1, run_indices)

	var calls := 50000
	var run_count := func() -> void:
		for i in calls:
			dt.get_triangle_count()
	_bench("call_overhead/get_triangle_count", 10, 1, calls, run_count)


# ── harness ───────────────────────────────────────────────────────────────────


func _bench(bench_name: String, iters: int, warmup: int, batch: int, f: Callable) -> void:
	for i in warmup:
		f.call()
	var samples := PackedFloat64Array()
	for i in iters:
		var t0 := Time.get_ticks_usec()
		f.call()
		samples.append(float(Time.get_ticks_usec() - t0))
	_report(bench_name, samples, batch)


@warning_ignore("integer_division")


func _report(bench_name: String, samples: PackedFloat64Array, batch: int) -> void:
	var s := samples.duplicate()
	s.sort()
	var mean := 0.0
	for v in samples:
		mean += v
	mean /= samples.size()
	var var_acc := 0.0
	for v in samples:
		var_acc += (v - mean) * (v - mean)
	var std := sqrt(var_acc / maxf(1.0, samples.size() - 1.0))
	var b := float(batch)
	print(
		(
			"%-36s mean %12.2f  median %12.2f  min %12.2f  std %10.2f  (n=%d)"
			% [bench_name, mean / b, s[s.size() / 2] / b, s[0] / b, std / b, samples.size()]
		)
	)


# ── datasets (must stay in sync with rust/rts-lib/benches) ────────────────────


# Same LCG as the criterion benches; >> 16 masked to emulate u64 / 65536.
func _lcg() -> int:
	_lcg_state = _lcg_state * 1103515245 + 12345
	return (_lcg_state >> 16) & 0xFFFFFFFFFFFF


func _random_points(count: int, point_seed: int) -> PackedVector2Array:
	_lcg_state = point_seed
	var pts := PackedVector2Array()
	pts.resize(count)
	for i in count:
		var x := float(_lcg() % 1000)
		var y := float(_lcg() % 1000)
		pts[i] = Vector2(x, y)
	return pts


# Edge between every 10th point and its successor (insert_chain_constraints).
func _chain_constraints(n: int) -> PackedInt32Array:
	var cons := PackedInt32Array()
	for i in range(0, n - 1, 10):
		cons.append(i)
		cons.append(i + 1)
	return cons


# Grid of square rooms with centred door gaps (port of common::rooms_map).
func _rooms_map(cols: int, rows: int) -> Array:
	var room := 100.0
	var door := 30.0
	var jamb_lo := (room - door) / 2.0
	var jamb_hi := (room + door) / 2.0
	# Plain Arrays: packed arrays are value types and would not mutate through
	# the _seg parameter.
	var points: Array = []
	var index := {}
	var cons: Array = []

	for i in cols + 1:
		var x := i * room
		for j in rows:
			var y := j * room
			if i == 0 or i == cols:
				_seg(Vector2(x, y), Vector2(x, y + room), points, index, cons)
			else:
				_seg(Vector2(x, y), Vector2(x, y + jamb_lo), points, index, cons)
				_seg(Vector2(x, y + jamb_hi), Vector2(x, y + room), points, index, cons)
	for j in rows + 1:
		var y := j * room
		for i in cols:
			var x := i * room
			if j == 0 or j == rows:
				_seg(Vector2(x, y), Vector2(x + room, y), points, index, cons)
			else:
				_seg(Vector2(x, y), Vector2(x + jamb_lo, y), points, index, cons)
				_seg(Vector2(x + jamb_hi, y), Vector2(x + room, y), points, index, cons)

	return [PackedVector2Array(points), PackedInt32Array(cons)]


func _seg(p: Vector2, q: Vector2, points: Array, index: Dictionary, cons: Array) -> void:
	cons.append(_point_id(p, points, index))
	cons.append(_point_id(q, points, index))


func _point_id(v: Vector2, points: Array, index: Dictionary) -> int:
	if not index.has(v):
		index[v] = points.size()
		points.append(v)
	return index[v]


# N evenly-spaced (start, goal) centroid pairs, goals half the face count away.
@warning_ignore("integer_division")


func _make_pairs(dt: DelaunayTriangulator, count: int) -> Array:
	var mesh: PackedVector2Array = dt.get_mesh_vertices()
	var nf := mesh.size() / 3
	var starts := PackedVector2Array()
	var goals := PackedVector2Array()
	for i in count:
		var sf := (i * nf) / count
		var gf := (sf + nf / 2) % nf
		starts.append((mesh[3 * sf] + mesh[3 * sf + 1] + mesh[3 * sf + 2]) / 3.0)
		goals.append((mesh[3 * gf] + mesh[3 * gf + 1] + mesh[3 * gf + 2]) / 3.0)
	return [starts, goals]
