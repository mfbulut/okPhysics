
package sample

import phy "physics"

import rl "vendor:raylib"

SCREEN_WIDTH  :: 1280
SCREEN_HEIGHT :: 720

World :: struct {
    bodies : [dynamic]phy.RigidBody,
    joints : [dynamic]phy.Joint,
    mouse_anchor: phy.Anchor,
}

main :: proc() {
    rl.SetConfigFlags({.VSYNC_HINT, .MSAA_4X_HINT})
    rl.InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics")

    world : World

    init_world(&world)

    for !rl.WindowShouldClose() {
        dt := rl.GetFrameTime()

        handle_mouse(&world, dt)
        update_world(&world, dt)

        rl.BeginDrawing()
        rl.ClearBackground(rl.Color{18, 18, 36, 255})

        draw_world(&world)

        rl.EndDrawing()
    }
}

init_world :: proc(world: ^World) {
    append(&world.bodies,
        phy.create_box({SCREEN_WIDTH/2, SCREEN_HEIGHT - 25}, SCREEN_WIDTH, 50, 0),

        phy.create_box({500, 100}, 40, 40, 1.0),
        phy.create_box({600, 100}, 40, 40, 1.0),
        phy.create_box({700, 100}, 40, 40, 1.0),
        phy.create_triangle({800, 200}, 25, 1.0),
        phy.create_triangle({900, 200}, 25, 1.0),
    )

    chain_links := 5

    for i in 0..<chain_links {
        x := 100 + f32(i) * 60
        append(&world.bodies, phy.create_box({x, 200}, 30, 30, 1.0))
    }

    for i in 0..<chain_links-1 {
        body_a := &world.bodies[len(world.bodies) - chain_links + i]
        body_b := &world.bodies[len(world.bodies) - chain_links + i + 1]

        anchor_a := body_a.position + phy.Vector2{15, 0}
        anchor_b := body_b.position + phy.Vector2{-15, 0}

        joint := phy.create_distance_joint(body_a, body_b, anchor_a, anchor_b, 400.0, 2.0)
        append(&world.joints, joint)
    }
}

update_world :: proc(world: ^World, dt: f32) {
    for &body in world.bodies {
        phy.apply_force(&body, phy.Vector2{0, 980} * body.mass * dt)
        phy.integrate(&body, dt)
    }

    for &joint in world.joints {
        phy.solve_joint(&joint, dt)
    }

    for &bodyA, i in world.bodies {
        for &bodyB in world.bodies[i+1:] {
            manifold := phy.collide(&bodyA, &bodyB)
            phy.resolve(manifold)
        }
    }
}

draw_world :: proc(world: ^World) {
    for &body in world.bodies {
        verticies := body.transformed_verticies
        count := i32(len(verticies))

        rl.DrawTriangleFan(&verticies[0], count, rl.GRAY)
        rl.DrawLineV(verticies[0], verticies[count - 1], rl.WHITE)
        rl.DrawLineStrip(&verticies[0], count, rl.WHITE)
    }

    for &joint in world.joints {
        if !joint.enabled do continue

        pos_a := phy.anchor_position(joint.anchor_a)
        pos_b := phy.anchor_position(joint.anchor_b)

        rl.DrawLineV(pos_a, pos_b, rl.YELLOW)
        rl.DrawCircleV(pos_a, 3, rl.YELLOW)
        rl.DrawCircleV(pos_b, 3, rl.YELLOW)
    }

    mouse_pos := rl.GetMousePosition()

    if world.mouse_anchor.body != nil {
        rl.DrawLineV(phy.anchor_position(world.mouse_anchor), mouse_pos, rl.Color{255, 0, 0, 255})
        rl.DrawCircleV(mouse_pos, 3, rl.Color{255, 0, 0, 100})
    }
}

handle_mouse :: proc(world: ^World, dt : f32) {
    mouse_pos := rl.GetMousePosition()

    if rl.IsMouseButtonPressed(.LEFT) {
        for &body in world.bodies {
            if phy.point_in_rigidbody(mouse_pos, &body) {
                world.mouse_anchor = phy.create_anchor(&body, mouse_pos)
                break
            }
        }
    }

    if rl.IsMouseButtonReleased(.LEFT) {
        world.mouse_anchor = {}
    }

    if world.mouse_anchor.body != nil {
        pos := phy.anchor_position(world.mouse_anchor)
        displacement := mouse_pos - pos
        phy.apply_force_at_point(world.mouse_anchor.body, displacement * 400 * dt, pos)

        world.mouse_anchor.body.velocity *= 0.9
        world.mouse_anchor.body.angular_velocity *= 0.95
    }

    if rl.IsMouseButtonPressed(.RIGHT) {
        new_body := phy.create_box(mouse_pos, 30, 30, 1.0)
        new_body.restitution = 0.7
        append(&world.bodies, new_body)
    }
}