use nannou::prelude::*;
use crate::types::{Model, RobotState, SimulationParams, Obstacle};

pub fn create_model(app: &App) -> Model {
    app.new_window().size(800, 800).view(crate::view::view).build().unwrap();
    
    let params = SimulationParams {
        v_max: 3.0,
        k_p: 20.0,
        decel_radius: 1.0,
        goal_threshold: 0.1,
        stop_threshold: 0.01,
        min_velocity: 0.1,
        pure_pursuit_lookahead: 0.5,
    };

    let vector_field_robot = RobotState {
        x: -3.0,
        y: -3.0,
        theta: 0.0,
    };

    let pure_pursuit_robot = RobotState {
        x: -3.0,
        y: -3.0,
        theta: 0.0,
    };


    let path_control_points = [
        pt2(-3.0, -3.0),
        pt2(-1.0, 1.0),
        pt2(1.0, -1.0),
        pt2(3.0, 3.0),
    ];

    let goal = path_control_points[3];

    let obstacles = vec![
        Obstacle { position: pt2(-3.0, 3.0), radius: 0.2 },
        Obstacle { position: pt2(3.0, -3.0), radius: 0.2 },
        Obstacle { position: pt2(0.0, 0.0), radius: 0.2 },
        Obstacle { position: pt2(0.0, 3.0), radius: 0.2 },
        Obstacle { position: pt2(0.0, -3.0), radius: 0.2 },
        Obstacle { position: pt2(3.0, 0.0), radius: 0.2 },
        Obstacle { position: pt2(-3.0, 0.0), radius: 0.2 },
        Obstacle { position: pt2(-3.0, -3.0), radius: 0.2 },
        Obstacle { position: pt2(3.0, 3.0), radius: 0.2 },
    ];

    let mut model = Model {
        vector_field_robot,
        pure_pursuit_robot,
        params,
        goal,
        vector_field_path: Vec::new(),
        pure_pursuit_path: Vec::new(),
        vf_reached_goal: false,
        pp_reached_goal: false,
        path: Vec::new(),
        path_control_points,
        simulation_time: 0.0,
        obstacles,
    };

    model.generate_bezier_path(100);

    model
}