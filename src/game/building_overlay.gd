## Draws placed buildings as filled boxes. Expects z_index 1 in the scene:
## above the navmesh floor/edges (z 0), below its constrained edges and points (z 2).
class_name BuildingOverlay
extends Node2D

@export var fill_color: Color = Color(1.0, 0.5, 0.0, 0.4):
	set(value):
		fill_color = value
		queue_redraw()

var _buildings: Array[Building] = []


class Building:
	var id: int
	var rect: Rect2  # World space

	func _init(id_: int, rect_: Rect2) -> void:
		id = id_
		rect = rect_


func add_building(id: int, rect: Rect2) -> void:
	_buildings.append(Building.new(id, rect))
	queue_redraw()


## Remove the topmost building containing `pos`; returns its id, or -1 if none.
func remove_building_at(pos: Vector2) -> int:
	for i in range(_buildings.size() - 1, -1, -1):
		if _buildings[i].rect.has_point(pos):
			var id := _buildings[i].id
			_buildings.remove_at(i)
			queue_redraw()
			return id
	return -1


## Remove all buildings; returns their ids.
func clear_buildings() -> Array[int]:
	var ids: Array[int] = []
	for building in _buildings:
		ids.append(building.id)
	_buildings.clear()
	queue_redraw()
	return ids


func _draw() -> void:
	for building in _buildings:
		draw_rect(building.rect, fill_color)
