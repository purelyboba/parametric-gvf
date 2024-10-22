use nannou::prelude::*;

pub struct RobotState {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

pub struct SimulationParams {
    pub v_max: f32,
    pub k_p: f32,
    pub decel_radius: f32,
    pub goal_threshold: f32,
    pub stop_threshold: f32,
    pub min_velocity: f32,
    pub pure_pursuit_lookahead: f32,
}

pub struct Obstacle {
    pub position: Point2,
    pub radius: f32,
}

pub struct Model {
    pub vector_field_robot: RobotState,
    pub pure_pursuit_robot: RobotState,
    pub params: SimulationParams,
    pub goal: Point2,  
    pub path: Vec<Point2>,
    pub path_control_points: [Point2; 4],
    pub vector_field_path: Vec<Point2>,
    pub pure_pursuit_path: Vec<Point2>,
    pub vf_reached_goal: bool,
    pub pp_reached_goal: bool,
    pub simulation_time: f32,
    pub obstacles: Vec<Obstacle>,
}