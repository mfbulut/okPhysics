package physics

import "core:math"
import "core:math/linalg"

Anchor :: struct {
	body:      ^RigidBody,
	local_pos: Vector2,
}

// TODO: Add more joint types
JointType :: enum {
	DISTANCE,
}

Joint :: struct {
	type:           JointType,
	body_a:         ^RigidBody,
	body_b:         ^RigidBody,
	anchor_a:       Anchor,
	anchor_b:       Anchor,
	length:         f32,
	stiffness:      f32,
	damping:        f32,
	enabled:        bool,
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

create_distance_joint :: proc(body_a, body_b: ^RigidBody, anchor_a, anchor_b: Vector2, stiffness: f32 = 1000.0, damping: f32 = 50.0) -> Joint {
	joint := Joint {
		type      = .DISTANCE,
		body_a    = body_a,
		body_b    = body_b,
		anchor_a  = create_anchor(body_a, anchor_a),
		anchor_b  = create_anchor(body_b, anchor_b),
		stiffness = stiffness,
		damping   = damping,
		enabled   = true,
	}

	pos_a := anchor_position(joint.anchor_a)
	pos_b := anchor_position(joint.anchor_b)
	joint.length = linalg.length(pos_b - pos_a)
	return joint
}

solve_joint :: proc(joint: ^Joint, dt: f32) {
	if !joint.enabled do return

	switch joint.type {
	case .DISTANCE:
		solve_distance_joint(joint, dt)
	}
}

solve_distance_joint :: proc(joint: ^Joint, dt: f32) {
	body_a := joint.body_a
	body_b := joint.body_b

	pos_a := anchor_position(joint.anchor_a)
	pos_b := anchor_position(joint.anchor_b)

	diff := pos_b - pos_a
	current_length := linalg.length(diff)

	if current_length == 0 do return

	direction := diff / current_length

	stretch := current_length - joint.length
	if stretch <= 0 do return  // Only pull when stretched
	spring_force := stretch * joint.stiffness

	vel_a := body_a.velocity
	vel_b := body_b.velocity
	relative_vel := vel_b - vel_a
	damping_force := linalg.dot(relative_vel, direction) * joint.damping

	total_force := (spring_force + damping_force) * dt
	force_vector := direction * total_force

	apply_force_at_point(body_a, force_vector, pos_a)
	apply_force_at_point(body_b, -force_vector, pos_b)
}