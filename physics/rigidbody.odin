package physics

import "core:math"
import "core:math/linalg"

Vector2 :: [2]f32

RigidBody :: struct {
	verticies:             []Vector2,
	transformed_verticies: []Vector2,
	normals:               []Vector2,
	position:              Vector2,
	velocity:              Vector2,
	rotation:              f32,
	angular_velocity:      f32,
	mass:                  f32,
	inv_mass:              f32,
	moment_of_inertia:     f32,
	inv_moment:            f32,
	restitution:           f32,
	friction:              f32,
}

CollisionManifold :: struct {
	body_a, body_b: ^RigidBody,
	point: Vector2,
	normal: Vector2,
	depth: f32,
	collision: bool,
}

SupportPoint :: struct {
	vertex: Vector2,
	depth: f32,
	valid: bool,
}

create_rigidbody :: proc(points: []Vector2, position: Vector2, mass: f32 = 1, restitution: f32 = 0, friction: f32 = 0.5) -> RigidBody {
	rb := RigidBody {
		position              = position,
		mass                  = mass,
		inv_mass              = mass > 0 ? 1.0 / mass : 0,
		verticies             = make([]Vector2, len(points)),
		transformed_verticies = make([]Vector2, len(points)),
		normals               = make([]Vector2, len(points)),
		restitution           = restitution,
		friction              = friction,
	}

	copy(rb.verticies, points)

	rb.moment_of_inertia = calculate_moment_of_inertia(&rb)
	rb.inv_moment = rb.moment_of_inertia > 0 ? 1.0 / rb.moment_of_inertia : 0

	transform_verticies(&rb)

	return rb
}

calculate_moment_of_inertia :: proc(rb: ^RigidBody) -> f32 {
	if rb.mass <= 0 do return 0

	moment: f32 = 0
	for point in rb.verticies {
		moment += point.x * point.x + point.y * point.y
	}

	return rb.mass * moment / f32(len(rb.verticies))
}

transform_verticies :: proc(rb: ^RigidBody) {
	cos_rot := math.cos(rb.rotation)
	sin_rot := math.sin(rb.rotation)

	for vert, i in rb.verticies {
		rotated_x := vert.x * cos_rot - vert.y * sin_rot
		rotated_y := vert.x * sin_rot + vert.y * cos_rot

		rb.transformed_verticies[i] = Vector2{rb.position.x + rotated_x, rb.position.y + rotated_y}
	}

	for i in 0 ..< len(rb.transformed_verticies) {
		current := rb.transformed_verticies[i]
		next := rb.transformed_verticies[(i + 1) % len(rb.transformed_verticies)]

		normal := Vector2{current.y - next.y, next.x - current.x}

		rb.normals[i] = linalg.normalize(normal)
	}
}

integrate :: proc(rb: ^RigidBody, dt: f32) {
    rb.position += rb.velocity * dt
    rb.rotation += rb.angular_velocity * dt

    transform_verticies(rb)
}

find_support_point :: proc(point: Vector2, normal: Vector2, verticies: []Vector2) -> SupportPoint {
	support: SupportPoint

	for vert in verticies {
		penetration := linalg.dot(vert - point, -normal)

		if penetration > support.depth {
			support.vertex = vert
			support.depth = penetration
			support.valid = true
		}
	}

	return support
}

get_contact_point :: proc(body_a, body_b: ^RigidBody) -> CollisionManifold {
	result: CollisionManifold

	min_depth := f32(1e6)

	for point, i in body_a.transformed_verticies {
		normal := body_a.normals[i]

		support := find_support_point(point, normal, body_b.transformed_verticies)
		if !support.valid {
			return {}
		}

		if support.depth < min_depth {
			min_depth = support.depth
			result.point = support.vertex
			result.normal = normal
			result.depth = min_depth
		}
	}

	result.collision = true
	return result
}

collide :: proc(body_a, body_b: ^RigidBody) -> CollisionManifold {
	manifold: CollisionManifold

	contact_a := get_contact_point(body_a, body_b)
	if !contact_a.collision {
		return manifold
	}

	contact_b := get_contact_point(body_b, body_a)
	if !contact_b.collision {
		return manifold
	}

	if contact_a.depth < contact_b.depth {
		offset := contact_a.normal * contact_a.depth
		manifold.point = contact_a.point - offset
		manifold.normal = contact_a.normal
		manifold.depth = contact_a.depth
	} else {
		manifold.point = contact_b.point
		manifold.normal = -contact_b.normal
		manifold.depth = contact_b.depth
	}

	manifold.body_a = body_a
	manifold.body_b = body_b
	manifold.collision = true

	return manifold
}

resolve :: proc(manifold: CollisionManifold) {
    a, b := manifold.body_a, manifold.body_b
    if a.inv_mass == 0 && b.inv_mass == 0 do return

    r_a := manifold.point - a.position
    r_b := manifold.point - b.position

    perp :: proc(r: Vector2) -> Vector2 { return {-r.y, r.x} }

    rel_vel := (b.velocity + perp(r_b) * b.angular_velocity) - (a.velocity + perp(r_a) * a.angular_velocity)

    vel_along_normal := linalg.dot(rel_vel, manifold.normal)
    if vel_along_normal > 0 do return

    e := min(a.restitution, b.restitution)
    r_a_cross := linalg.cross(r_a, manifold.normal)
    r_b_cross := linalg.cross(r_b, manifold.normal)

    inv_mass_sum := a.inv_mass + b.inv_mass
    angular_term := r_a_cross * r_a_cross * a.inv_moment + r_b_cross * r_b_cross * b.inv_moment

    j := -(1 + e) * vel_along_normal / (inv_mass_sum + angular_term)
    impulse := manifold.normal * j

    a.velocity -= impulse * a.inv_mass
    b.velocity += impulse * b.inv_mass
    a.angular_velocity -= r_a_cross * j * a.inv_moment
    b.angular_velocity += r_b_cross * j * b.inv_moment

    rel_vel = (b.velocity + perp(r_b) * b.angular_velocity) - (a.velocity + perp(r_a) * a.angular_velocity)

    tangent := rel_vel - manifold.normal * linalg.dot(rel_vel, manifold.normal)
    tangent = linalg.normalize0(tangent)

    r_a_cross_t := linalg.cross(r_a, tangent)
    r_b_cross_t := linalg.cross(r_b, tangent)
    angular_tan_term := r_a_cross_t * r_a_cross_t * a.inv_moment + r_b_cross_t * r_b_cross_t * b.inv_moment

    jt := -linalg.dot(rel_vel, tangent) * min(a.friction, b.friction)
    jt /= (inv_mass_sum + angular_tan_term)
    jt = min(jt, j)

    friction_impulse := tangent * jt

    a.velocity -= friction_impulse * a.inv_mass
    b.velocity += friction_impulse * b.inv_mass
    a.angular_velocity -= r_a_cross_t * jt * a.inv_moment
    b.angular_velocity += r_b_cross_t * jt * b.inv_moment

    correction := manifold.normal * (manifold.depth / (a.inv_mass + b.inv_mass) * 0.9)

    a.position -= correction * a.inv_mass
    b.position += correction * b.inv_mass

    transform_verticies(a)
    transform_verticies(b)
}

apply_force :: proc(rb: ^RigidBody, force: Vector2) {
	rb.velocity += force * rb.inv_mass
}

apply_torque :: proc(rb: ^RigidBody, torque: f32) {
	rb.angular_velocity += torque * rb.inv_moment
}

apply_force_at_point :: proc(rb: ^RigidBody, force: Vector2, point: Vector2) {
	r := point - rb.position
	torque := r.x * force.y - r.y * force.x
	apply_torque(rb, torque)
	apply_force(rb, force)
}