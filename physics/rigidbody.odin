
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
	contact_point:  Vector2,
	contact_normal: Vector2,
	depth:          f32,
	collision:      bool,
}

SupportPoint :: struct {
	vertex: Vector2,
	depth:  f32,
	valid:  bool,
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

	update_transformed_verticies(&rb)

	return rb
}

@(private)
calculate_moment_of_inertia :: proc(rb: ^RigidBody) -> f32 {
	if rb.mass <= 0 do return 0

	moment: f32 = 0
	for point in rb.verticies {
		r_squared := point.x * point.x + point.y * point.y
		moment += r_squared
	}

	return rb.mass * moment / f32(len(rb.verticies))
}

@(private)
update_transformed_verticies :: proc(rb: ^RigidBody) {
	cos_rot := math.cos(rb.rotation)
	sin_rot := math.sin(rb.rotation)

	for i in 0 ..< len(rb.verticies) {
		local := rb.verticies[i]

		rotated_x := local.x * cos_rot - local.y * sin_rot
		rotated_y := local.x * sin_rot + local.y * cos_rot

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

	update_transformed_verticies(rb)
}

@(private)
find_support_point :: proc(point: Vector2, normal: Vector2, verticies: []Vector2) -> SupportPoint {
	support: SupportPoint

	for vert in verticies {
		vert_to_point := vert - point
		penetration := linalg.dot(vert_to_point, -normal)

		if penetration > support.depth {
			support.vertex = vert
			support.depth = penetration
			support.valid = true
		}
	}

	return support
}

@(private)
get_contact_point :: proc(body_a, body_b: ^RigidBody) -> CollisionManifold {
	result: CollisionManifold

	min_depth := f32(1e6)

	for i in 0 ..< len(body_a.transformed_verticies) {
		point := body_a.transformed_verticies[i]
		normal := body_a.normals[i]

		support := find_support_point(point, normal, body_b.transformed_verticies)
		if !support.valid {
			return result
		}

		if support.depth < min_depth {
			min_depth = support.depth
			result.contact_point = support.vertex
			result.contact_normal = normal
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
		offset := contact_a.contact_normal * contact_a.depth
		manifold.contact_point = contact_a.contact_point - offset
		manifold.contact_normal = contact_a.contact_normal
		manifold.depth = contact_a.depth
	} else {
		manifold.contact_point = contact_b.contact_point
		manifold.contact_normal = -contact_b.contact_normal
		manifold.depth = contact_b.depth
	}

	manifold.body_a = body_a
	manifold.body_b = body_b
	manifold.collision = true
	return manifold
}

resolve :: proc(manifold: CollisionManifold) {
	if !manifold.collision do return

	rigidbodyA := manifold.body_a
	rigidbodyB := manifold.body_b

	if rigidbodyA.inv_mass == 0 && rigidbodyB.inv_mass == 0 {
		return
	}

	penetrationToCentroidA := manifold.contact_point - rigidbodyA.position
	penetrationToCentroidB := manifold.contact_point - rigidbodyB.position

	angularVelocityPenetrationCentroidA := Vector2 {
		-rigidbodyA.angular_velocity * penetrationToCentroidA.y,
		rigidbodyA.angular_velocity * penetrationToCentroidA.x,
	}

	angularVelocityPenetrationCentroidB := Vector2 {
		-rigidbodyB.angular_velocity * penetrationToCentroidB.y,
		rigidbodyB.angular_velocity * penetrationToCentroidB.x,
	}

	relativeVelocityA := rigidbodyA.velocity + angularVelocityPenetrationCentroidA
	relativeVelocityB := rigidbodyB.velocity + angularVelocityPenetrationCentroidB

	relativeVel := relativeVelocityB - relativeVelocityA
	velocityInNormal := linalg.dot(relativeVel, manifold.contact_normal)

	if velocityInNormal > 0 {
		return
	}

	e := min(rigidbodyA.restitution, rigidbodyB.restitution)

	pToCentroidCrossNormalA :=
		penetrationToCentroidA.x * manifold.contact_normal.y -
		penetrationToCentroidA.y * manifold.contact_normal.x
	pToCentroidCrossNormalB :=
		penetrationToCentroidB.x * manifold.contact_normal.y -
		penetrationToCentroidB.y * manifold.contact_normal.x

	invMassSum := rigidbodyA.inv_mass + rigidbodyB.inv_mass
	crossNSum :=
		pToCentroidCrossNormalA * pToCentroidCrossNormalA * rigidbodyA.inv_moment +
		pToCentroidCrossNormalB * pToCentroidCrossNormalB * rigidbodyB.inv_moment

	j := -(1 + e) * velocityInNormal
	j /= (invMassSum + crossNSum)

	impulseVector := manifold.contact_normal * j

	rigidbodyA.velocity -= impulseVector * rigidbodyA.inv_mass
	rigidbodyB.velocity += impulseVector * rigidbodyB.inv_mass

	rigidbodyA.angular_velocity += -pToCentroidCrossNormalA * j * rigidbodyA.inv_moment
	rigidbodyB.angular_velocity += pToCentroidCrossNormalB * j * rigidbodyB.inv_moment

	// Friction
	velocityInNormalDirection := manifold.contact_normal * linalg.dot(relativeVel, manifold.contact_normal)
	tangent := relativeVel - velocityInNormalDirection

	if linalg.length(tangent) > 0.00001 {
		tangent = linalg.normalize(tangent)
	} else {
		tangent = Vector2{0, 0}
	}

	minFriction := min(rigidbodyA.friction, rigidbodyB.friction)

	pToCentroidCrossTangentA := penetrationToCentroidA.x * tangent.y - penetrationToCentroidA.y * tangent.x
	pToCentroidCrossTangentB := penetrationToCentroidB.x * tangent.y - penetrationToCentroidB.y * tangent.x

	crossSumTangent :=
		pToCentroidCrossTangentA * pToCentroidCrossTangentA * rigidbodyA.inv_moment +
		pToCentroidCrossTangentB * pToCentroidCrossTangentB * rigidbodyB.inv_moment

	frictionalImpulse := -linalg.dot(relativeVel, tangent) * minFriction
	frictionalImpulse /= (invMassSum + crossSumTangent)

	if frictionalImpulse > j {
		frictionalImpulse = j
	}

	frictionalImpulseVector := tangent * frictionalImpulse

	rigidbodyA.velocity -= frictionalImpulseVector * rigidbodyA.inv_mass
	rigidbodyB.velocity += frictionalImpulseVector * rigidbodyB.inv_mass

	rigidbodyA.angular_velocity += -pToCentroidCrossTangentA * frictionalImpulse * rigidbodyA.inv_moment
	rigidbodyB.angular_velocity += pToCentroidCrossTangentB * frictionalImpulse * rigidbodyB.inv_moment

	// Positional correction
	correctionPercentage: f32 = 0.9
	amountToCorrect  := (manifold.depth / (rigidbodyA.inv_mass + rigidbodyB.inv_mass)) * correctionPercentage
	correctionVector := manifold.contact_normal * amountToCorrect

	rigidbodyA.position -= correctionVector * rigidbodyA.inv_mass
	rigidbodyB.position += correctionVector * rigidbodyB.inv_mass

	update_transformed_verticies(rigidbodyA)
	update_transformed_verticies(rigidbodyB)
}

apply_force :: proc(rb: ^RigidBody, force: Vector2) {
	rb.velocity += force * rb.inv_mass
}

apply_torque :: proc(rb: ^RigidBody, torque: f32) {
	rb.angular_velocity += torque * rb.inv_moment
}

apply_force_at_point :: proc(rb: ^RigidBody, force: Vector2, point: Vector2) {
	apply_force(rb, force)

	r := Vector2{point.x - rb.position.x, point.y - rb.position.y}
	torque := r.x * force.y - r.y * force.x
	apply_torque(rb, torque)
}