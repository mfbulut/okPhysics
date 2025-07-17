package physics

point_in_rigidbody :: proc(point: Vector2, body: ^RigidBody) -> bool {
	count := 0
	vertices := body.transformed_verticies
	n := len(vertices)

	for i in 0..<n {
		j := (i + 1) % n
		if ((vertices[i].y > point.y) != (vertices[j].y > point.y)) &&
		   (point.x < (vertices[j].x - vertices[i].x) * (point.y - vertices[i].y) / (vertices[j].y - vertices[i].y) + vertices[i].x) {
			count += 1
		}
	}

	return count % 2 == 1
}

create_box :: proc(pos: Vector2, width, height, mass: f32) -> RigidBody {
	hw := width * 0.5
	hh := height * 0.5

	points := []Vector2{{-hw, hh}, {hw, hh}, {hw, -hh}, {-hw, -hh}}
	return create_rigidbody(points, pos, mass)
}

create_triangle :: proc(pos: Vector2, size, mass: f32) -> RigidBody {
	points := []Vector2{{-size, size}, {size, size}, {0, -size}}
	return create_rigidbody(points, pos, mass, 0.5, 0.3)
}
