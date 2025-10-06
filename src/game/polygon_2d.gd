class_name EdgeDrawPolygon2D
extends Polygon2D

@export var outline_color: Color = Color(0, 0, 1)

func _draw() -> void:
	for poly in polygons:
		draw_polyline(PackedVector2Array([polygon[poly[0]], polygon[poly[1]], polygon[poly[2]], polygon[poly[0]]]), outline_color)
