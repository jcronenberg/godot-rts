extends MainLoop

var suites = [
	["DelaunayTriangulator", preload("res://tests/test_delaunay.gd").new()],
]

func _initialize() -> void:
	print("=== RTS GDExtension Test Suite ===\n")

	var results: Array = []
	var total_passed := 0
	var total_failed := 0

	for entry in suites:
		var name: String = entry[0]
		var module = entry[1]
		print("--- %s ---" % name)
		var result: Dictionary = module.test()
		results.append([name, result])
		total_passed += result["passed"]
		total_failed += result["failed"]
		print("")

	_print_summary(results, total_passed, total_failed)


func _process(_delta: float) -> bool:
	return true

func _print_summary(results: Array, total_passed: int, total_failed: int) -> void:
	print("=== Summary ===")
	for entry in results:
		var name: String = entry[0]
		var result: Dictionary = entry[1]
		var passed: int = result["passed"]
		var total: int = result["passed"] + result["failed"]
		print("  %-30s %d/%d" % [name, passed, total])
	print("Total: %d passed, %d failed" % [total_passed, total_failed])
