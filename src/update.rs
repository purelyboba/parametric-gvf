use nannou::prelude::*;
use crate::types::Model;

pub fn update(_app: &App, model: &mut Model, update: Update) {
    let dt = update.since_last.as_secs_f32();
    
    // Update vector field robot
    if !model.vf_reached_goal {
        let (v, omega) = model.compute_control();
        
        model.vector_field_robot.x += v * model.vector_field_robot.theta.cos() * dt;
        model.vector_field_robot.y += v * model.vector_field_robot.theta.sin() * dt;
        model.vector_field_robot.theta += omega * dt;

        model.vector_field_path.push(pt2(model.vector_field_robot.x, 
                                       model.vector_field_robot.y));

        let vf_distance = model.distance_to_goal();
        if vf_distance < model.params.stop_threshold {
            model.vf_reached_goal = true;
            model.vector_field_robot.x = model.goal.x;
            model.vector_field_robot.y = model.goal.y;
            model.vector_field_path.push(model.goal);
            println!("Vector Field robot reached goal in {:.2} seconds!", 
                    model.simulation_time);
        }
    }
    
    // Update pure pursuit robot
    if !model.pp_reached_goal {
        let (v, omega) = model.compute_pure_pursuit_control();
        
        model.pure_pursuit_robot.x += v * model.pure_pursuit_robot.theta.cos() * dt;
        model.pure_pursuit_robot.y += v * model.pure_pursuit_robot.theta.sin() * dt;
        model.pure_pursuit_robot.theta += omega * dt;

        model.pure_pursuit_path.push(pt2(model.pure_pursuit_robot.x, 
                                       model.pure_pursuit_robot.y));

        let pp_distance = ((model.goal.x - model.pure_pursuit_robot.x).powi(2) + 
                          (model.goal.y - model.pure_pursuit_robot.y).powi(2)).sqrt();
                          
        if pp_distance < model.params.stop_threshold {
            model.pp_reached_goal = true;
            model.pure_pursuit_robot.x = model.goal.x;
            model.pure_pursuit_robot.y = model.goal.y;
            model.pure_pursuit_path.push(model.goal);
            println!("Pure Pursuit robot reached goal in {:.2} seconds!", 
                    model.simulation_time);
        }
    }

    model.simulation_time += dt;
}