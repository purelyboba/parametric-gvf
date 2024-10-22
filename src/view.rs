use nannou::prelude::*;
use std::f32::consts::PI;
use crate::types::Model;

pub fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    
    draw.background().color(WHITE);

    let scaling = 100.0;
    
    let grid_resolution = 20;
    let field_size = 8.0;
    let step = field_size / grid_resolution as f32;
    
    for i in 0..grid_resolution {
        for j in 0..grid_resolution {
            let x = -field_size / 2.0 + i as f32 * step;
            let y = -field_size / 2.0 + j as f32 * step;
            
            let point = pt2(x, y);
            
            let mut within_obstacle_influence = false;
            for obstacle in &model.obstacles {
                if (point - obstacle.position).length() <= obstacle.radius {
                    within_obstacle_influence = true;
                    break;
                }
            }
            if within_obstacle_influence {
                continue; 
            }
            
            let distance_to_goal = (point - model.goal).length();
            
            let (dx, dy) = if distance_to_goal <= model.params.decel_radius {
                (model.goal.x - x, model.goal.y - y)
            } else {
                model.calculate_vector_field(x, y)
            };
            
            if !dx.is_finite() || !dy.is_finite() {
                continue;
            }
            
            let magnitude = (dx * dx + dy * dy).sqrt();
            
            let arrow_length = 0.15;
            let start = point * scaling;
            let direction = if magnitude > 0.0 {
                pt2(dx, dy) / magnitude
            } else {
                pt2(0.0, 0.0)
            };
            let end = (point + direction * arrow_length) * scaling;

            let max_magnitude = 1.0;
            let normalized_magnitude = (magnitude / max_magnitude).min(1.0);
            let color = rgb(
                0.5 - normalized_magnitude * 0.5,
                0.5 - normalized_magnitude * 0.3,
                1.0 - normalized_magnitude * 0.3
            );

            draw.line()
                .start(start)
                .end(end)
                .stroke_weight(1.0)
                .color(color);
            
            let arrow_head_length = 0.04 * scaling;
            let angle = dy.atan2(dx);
            let head_angle = PI / 6.0;
            
            let head1 = pt2(
                end.x - arrow_head_length * (angle + head_angle).cos(),
                end.y - arrow_head_length * (angle + head_angle).sin()
            );
            let head2 = pt2(
                end.x - arrow_head_length * (angle - head_angle).cos(),
                end.y - arrow_head_length * (angle - head_angle).sin()
            );
            
            draw.line()
                .start(end)
                .end(head1)
                .stroke_weight(1.0)
                .color(color);
            
            draw.line()
                .start(end)
                .end(head2)
                .stroke_weight(1.0)
                .color(color);
        }
    }

    for obstacle in &model.obstacles {
        draw.ellipse()
            .xy(obstacle.position * scaling)
            .radius(obstacle.radius * scaling)
            .color(rgba(1.0, 0.0, 0.0, 0.5));
    }
    
    draw.ellipse()
        .xy(model.goal * scaling)
        .radius(model.params.decel_radius * scaling)
        .color(rgba(1.0, 1.0, 0.0, 0.1));

    draw.ellipse()
        .xy(model.goal * scaling)
        .radius(model.params.goal_threshold * scaling)
        .color(rgba(0.0, 1.0, 0.0, 0.3));

        if model.vector_field_path.len() >= 2 {
            draw.polyline()
                .points(model.vector_field_path.iter().map(|p| *p * scaling))
                .color(BLUE);
        }
    
        if model.pure_pursuit_path.len() >= 2 {
            draw.polyline()
                .points(model.pure_pursuit_path.iter().map(|p| *p * scaling))
                .color(RED);
        }
    
        if model.path.len() >= 2 {
            draw.polyline()
                .points(model.path.iter().map(|p| *p * scaling))
                .color(rgba(1.0, 0.0, 1.0, 0.5));
        }
    
        // Draw vector field robot
        let vf_robot_pos = pt2(model.vector_field_robot.x, 
                              model.vector_field_robot.y) * scaling;
        draw.ellipse()
            .xy(vf_robot_pos)
            .radius(10.0)
            .color(BLUE);
        
        let vf_heading_end = vf_robot_pos + pt2(
            model.vector_field_robot.theta.cos() * 20.0,
            model.vector_field_robot.theta.sin() * 20.0
        );
        draw.line()
            .start(vf_robot_pos)
            .end(vf_heading_end)
            .color(BLUE);
    
        // Draw pure pursuit robot
        let pp_robot_pos = pt2(model.pure_pursuit_robot.x, 
                              model.pure_pursuit_robot.y) * scaling;
        draw.ellipse()
            .xy(pp_robot_pos)
            .radius(10.0)
            .color(RED);
        
        let pp_heading_end = pp_robot_pos + pt2(
            model.pure_pursuit_robot.theta.cos() * 20.0,
            model.pure_pursuit_robot.theta.sin() * 20.0
        );
        draw.line()
            .start(pp_robot_pos)
            .end(pp_heading_end)
            .color(RED);
    
        // Draw lookahead point for pure pursuit
        if !model.pp_reached_goal {
            let lookahead = model.find_lookahead_point(
                pt2(model.pure_pursuit_robot.x, model.pure_pursuit_robot.y)
            );
            draw.ellipse()
                .xy(lookahead * scaling)
                .radius(5.0)
                .color(rgba(1.0, 0.0, 0.0, 0.5));
        }
    
        // Draw status text
        let time_text = format!("Time: {:.2}s", model.simulation_time);
        draw.text(&time_text)
            .xy(pt2(-550.0, 350.0))
            .color(BLACK)
            .font_size(20);
    
        let status_text = format!("Vector Field: {}\nPure Pursuit: {}", 
            if model.vf_reached_goal { "Goal reached" } else { "In progress" },
            if model.pp_reached_goal { "Goal reached" } else { "In progress" }
        );
        draw.text(&status_text)
            .xy(pt2(-550.0, 320.0))
            .color(BLACK)
            .font_size(20);
    
        // Draw legend
        draw.text("Vector Field")
            .xy(pt2(-550.0, 280.0))
            .color(BLUE)
            .font_size(20);
        draw.text("Pure Pursuit")
            .xy(pt2(-550.0, 250.0))
            .color(RED)
            .font_size(20);
    
        draw.to_frame(app, &frame).unwrap();
    }