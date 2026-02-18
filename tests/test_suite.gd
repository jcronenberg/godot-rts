class_name TestSuite
extends RefCounted

# Subclasses must override this to return [["test_name", callable], ...]
func get_cases() -> Array:
	return []

func test() -> Dictionary:
	var passed := 0
	var failed := 0

	for case in get_cases():
		var case_name: String = case[0]
		var failures: Array[String] = case[1].call()
		if failures.is_empty():
			print("  [PASS] %s" % case_name)
			passed += 1
		else:
			print("  [FAIL] %s" % case_name)
			for msg in failures:
				print("         %s" % msg)
			failed += 1

	return {"passed": passed, "failed": failed}
