package sample

import phy "physics"
import rl "vendor:raylib"

bodies : [dynamic; 2048]phy.RigidBody
joints : [dynamic; 2048]phy.Joint
mouse_anchor: phy.Anchor

main :: proc() {
    rl.SetConfigFlags({.VSYNC_HINT, .MSAA_4X_HINT})
    rl.InitWindow(1280, 720, "Physics")

    init_world()

    for !rl.WindowShouldClose() {
        dt := min(rl.GetFrameTime(), 0.05)

        handle_mouse(dt)
        update_world(dt)

        rl.BeginDrawing()
        rl.ClearBackground(rl.Color{10, 18, 30, 255})

        draw_world()

        rl.EndDrawing()
    }
}

init_world :: proc() {
    append(&bodies, phy.create_box({640, 690}, 1200, 30, 0))

    append(&bodies, phy.create_box({200, 400}, 100, 20, 1.0))
    anchor_hinge_a := phy.create_anchor(&bodies[0], {200, 400})
    anchor_hinge_b := phy.create_anchor(&bodies[1], {200, 400})
    append(&joints, phy.create_joint(anchor_hinge_a, anchor_hinge_b, .HINGE))

    append(&bodies, phy.create_box({400, 300}, 40, 40, 1.0))
    append(&bodies, phy.create_box({440, 300}, 40, 40, 1.0))
    anchor_fixed_a := phy.create_anchor(&bodies[2], {420, 300})
    anchor_fixed_b := phy.create_anchor(&bodies[3], {420, 300})
    append(&joints, phy.create_joint(anchor_fixed_a, anchor_fixed_b, .FIXED))

    append(&bodies, phy.create_box({600, 200}, 50, 50, 1.0))
    anchor_dist_a := phy.create_anchor(&bodies[0], {600, 50})
    anchor_dist_b := phy.create_anchor(&bodies[4], {600, 200})
    append(&joints, phy.create_joint(anchor_dist_a, anchor_dist_b, .FORCE))

    append(&bodies, phy.create_box({800, 100}, 30, 60, 1.0))
    append(&bodies, phy.create_box({800, 180}, 30, 60, 1.0))
    append(&bodies, phy.create_box({800, 260}, 30, 60, 1.0))

    a_chain_ground := phy.create_anchor(&bodies[0], {800, 70})
    a_chain_5_top  := phy.create_anchor(&bodies[5], {800, 70})
    append(&joints, phy.create_joint(a_chain_ground, a_chain_5_top, .HINGE))

    a_chain_5_bot  := phy.create_anchor(&bodies[5], {800, 130})
    a_chain_6_top  := phy.create_anchor(&bodies[6], {800, 150})
    append(&joints, phy.create_joint(a_chain_5_bot, a_chain_6_top, .HINGE))

    a_chain_6_bot  := phy.create_anchor(&bodies[6], {800, 210})
    a_chain_7_top  := phy.create_anchor(&bodies[7], {800, 230})
    append(&joints, phy.create_joint(a_chain_6_bot, a_chain_7_top, .HINGE))

    append(&bodies, phy.create_box({1000, 400}, 200, 20, 1.0))

    a_bridge_l_ground := phy.create_anchor(&bodies[0], {900, 300})
    a_bridge_l_plat   := phy.create_anchor(&bodies[8], {900, 400})
    append(&joints, phy.create_joint(a_bridge_l_ground, a_bridge_l_plat, .FORCE))

    a_bridge_r_ground := phy.create_anchor(&bodies[0], {1100, 300})
    a_bridge_r_plat   := phy.create_anchor(&bodies[8], {1100, 400})
    append(&joints, phy.create_joint(a_bridge_r_ground, a_bridge_r_plat, .FORCE))
}

update_world :: proc(total_dt: f32) {
    substeps := 4
    sub_dt := total_dt / f32(substeps)

    for i in 0..<substeps {
        for &body in bodies {
            phy.apply_force(&body, phy.Vector2{0, 980} * body.mass * sub_dt)
        }

        for &body in bodies {
            phy.integrate(&body, sub_dt)
        }

        for &joint in joints {
            phy.solve_joint(&joint, sub_dt)
        }

        for &bodyA, i in bodies {
            for &bodyB in bodies[i+1:] {
                manifold := phy.collide(&bodyA, &bodyB)
                if manifold.collision {
                    phy.resolve(manifold)
                }
            }
        }
    }
}

draw_world :: proc() {
    for &body in bodies {
        verticies := body.transformed_verticies
        count := i32(len(verticies))

        rl.DrawTriangleFan(&verticies[0], count, rl.GRAY)
        rl.DrawLineV(verticies[0], verticies[count - 1], rl.WHITE)
        rl.DrawLineStrip(&verticies[0], count, rl.WHITE)
    }

    for &joint in joints {
        pos_a := phy.anchor_position(joint.anchor_a)
        pos_b := phy.anchor_position(joint.anchor_b)

        rl.DrawLineV(pos_a, pos_b, rl.YELLOW)
        rl.DrawCircleV(pos_a, 3, rl.YELLOW)
        rl.DrawCircleV(pos_b, 3, rl.YELLOW)
    }

    mouse_pos := rl.GetMousePosition()

    if mouse_anchor.body != nil {
        rl.DrawLineV(phy.anchor_position(mouse_anchor), mouse_pos, rl.Color{255, 0, 0, 255})
        rl.DrawCircleV(mouse_pos, 3, rl.Color{255, 0, 0, 100})
    }
}

handle_mouse :: proc(dt : f32) {
    mouse_pos := rl.GetMousePosition()

    if rl.IsMouseButtonPressed(.LEFT) {
        for &body in bodies {
            if phy.point_in_rigidbody(mouse_pos, &body) {
                mouse_anchor = phy.create_anchor(&body, mouse_pos)
                break
            }
        }
    }

    if rl.IsMouseButtonReleased(.LEFT) {
        mouse_anchor = {}
    }

    if mouse_anchor.body != nil {
        pos := phy.anchor_position(mouse_anchor)
        displacement := mouse_pos - pos
        phy.apply_force_at_point(mouse_anchor.body, displacement * 400 * dt, pos)

        mouse_anchor.body.velocity *= 0.9
        mouse_anchor.body.angular_velocity *= 0.95
    }

    if rl.IsMouseButtonPressed(.RIGHT) {
        new_body := phy.create_box(mouse_pos, 30, 30, 1.0)
        new_body.restitution = 0.7
        append(&bodies, new_body)
    }
}