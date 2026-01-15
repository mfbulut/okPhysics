package physics

import "core:math"
import "core:math/linalg"

Anchor :: struct {
	body:      ^RigidBody,
	local_pos: Vector2,
}

create_anchor :: proc(body: ^RigidBody, world_pos: Vector2) -> Anchor {
	relative_pos := world_pos - body.position

	cos_rot := math.cos(-body.rotation)
	sin_rot := math.sin(-body.rotation)

	local_pos := Vector2 {
		relative_pos.x * cos_rot - relative_pos.y * sin_rot,
		relative_pos.x * sin_rot + relative_pos.y * cos_rot,
	}

	anchor := Anchor {
		body      = body,
		local_pos = local_pos,
	}

	return anchor
}

anchor_position :: proc(anchor: Anchor) -> Vector2 {
	cos_rot := math.cos(anchor.body.rotation)
	sin_rot := math.sin(anchor.body.rotation)

	rotated_x := anchor.local_pos.x * cos_rot - anchor.local_pos.y * sin_rot
	rotated_y := anchor.local_pos.x * sin_rot + anchor.local_pos.y * cos_rot

	return Vector2 {
		anchor.body.position.x + rotated_x,
		anchor.body.position.y + rotated_y,
	}
}

JointType :: enum {
	FORCE,
	HINGE,
	FIXED,
}

Joint :: struct {
	type:                 JointType,
	anchor_a:             Anchor,
	anchor_b:             Anchor,
	initial_length:        f32,
	relative_orientation: f32,
	enabled:              bool,
}

create_joint :: proc(anchor_a: Anchor, anchor_b: Anchor, type: JointType) -> Joint {
	joint := Joint {
		type     = type,
		anchor_a = anchor_a,
		anchor_b = anchor_b,
		enabled  = true,
	}

	world_a := anchor_position(joint.anchor_a)
	world_b := anchor_position(joint.anchor_b)

	joint.initial_length = linalg.distance(world_a, world_b)
	joint.relative_orientation = anchor_b.body.rotation - anchor_a.body.rotation

	return joint
}

solve_force_constraint :: proc(joint: ^Joint, dt: f32) {
    anchor_a_pos := anchor_position(joint.anchor_a)
    anchor_b_pos := anchor_position(joint.anchor_b)

    delta := anchor_b_pos - anchor_a_pos
    distance := linalg.length(delta)
    if distance < 0.001 do return

    direction := delta / distance
    length_error := distance - joint.initial_length

    stiffness : f32 = 500.0
    spring_force := length_error * stiffness

    vel_a := joint.anchor_a.body.velocity +
             Vector2{-joint.anchor_a.body.angular_velocity * (anchor_a_pos.y - joint.anchor_a.body.position.y),
                      joint.anchor_a.body.angular_velocity * (anchor_a_pos.x - joint.anchor_a.body.position.x)}

    vel_b := joint.anchor_b.body.velocity +
             Vector2{-joint.anchor_b.body.angular_velocity * (anchor_b_pos.y - joint.anchor_b.body.position.y),
                      joint.anchor_b.body.angular_velocity * (anchor_b_pos.x - joint.anchor_b.body.position.x)}

    relative_velocity := vel_b - vel_a
    damping_factor : f32 = 20.0
    damping_force := linalg.dot(relative_velocity, direction) * damping_factor

    total_force := direction * (spring_force + damping_force) * dt

    apply_force_at_point(joint.anchor_a.body,  total_force, anchor_a_pos)
    apply_force_at_point(joint.anchor_b.body, -total_force, anchor_b_pos)
}

solve_hinge_constraint :: proc(joint: ^Joint, dt: f32) {
	anchor_a_pos := anchor_position(joint.anchor_a)
	anchor_b_pos := anchor_position(joint.anchor_b)

	anchor_dir := anchor_a_pos - anchor_b_pos
	distance := linalg.length(anchor_dir)

	if distance < 0.01 do return

	normal := linalg.normalize(anchor_dir)

	contact := CollisionManifold {
		normal      = normal,
		point       = anchor_b_pos,
		body_a      = joint.anchor_a.body,
		body_b      = joint.anchor_b.body,
		collision   = true,
	}

	if distance > joint.initial_length {
		contact.depth = distance - joint.initial_length
	} else {
		contact.depth = joint.initial_length - distance
		contact.normal = -contact.normal
	}

	resolve(contact)
}

solve_rotation_constraint :: proc(joint: ^Joint, dt: f32) {
    body_a := joint.anchor_a.body
    body_b := joint.anchor_b.body

    current_diff := body_b.rotation - body_a.rotation
    orientation_error := joint.relative_orientation - current_diff

    for orientation_error > math.PI  do orientation_error -= 2 * math.PI
    for orientation_error < -math.PI do orientation_error += 2 * math.PI

    bias : f32 = 0.2
    inv_inertia_sum := body_a.inv_moment + body_b.inv_moment
    if inv_inertia_sum > 0 {
        correction := (orientation_error / inv_inertia_sum) * bias
        body_a.rotation -= correction * body_a.inv_moment
        body_b.rotation += correction * body_b.inv_moment
    }

    average_angular_vel := (body_a.angular_velocity + body_b.angular_velocity) * 0.5
    body_a.angular_velocity = average_angular_vel
    body_b.angular_velocity = average_angular_vel
}

solve_joint :: proc(joint: ^Joint, dt: f32) {
	if !joint.enabled do return

	switch joint.type {
	case .FORCE:
		solve_force_constraint(joint, dt)
	case .HINGE:
		solve_hinge_constraint(joint, dt)
	case .FIXED:
		solve_hinge_constraint(joint, dt)
		solve_rotation_constraint(joint, dt)
	}
}
