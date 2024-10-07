use nannou::prelude::*;
use std::f32::consts::PI;

struct RobotState {
    x: f32,
    y: f32,
    theta: f32,
}

struct SimulationParams {
    // control params
    v_max: f32,
    k_p: f32,
    // movement params
    decel_radius: f32,
    goal_threshold: f32,
    stop_threshold: f32,
    min_velocity: f32,
    decel_power: f32,
}

// state
struct Model {
    robot: RobotState,
    params: SimulationParams,
    goal: Point2,
    path: Vec<Point2>,
    reached_goal: bool,
}

fn main() {
    // run visualization
    nannou::app(model)
        .update(update)
        .run();
}

// init model
fn model(app: &App) -> Model {
    app.new_window().size(800, 800).view(view).build().unwrap();
    
    let params = SimulationParams {
        v_max: 1.0,
        k_p: 2.0,
        decel_radius: 2.0,
        goal_threshold: 0.2,
        stop_threshold: 0.05,
        min_velocity: 0.3,
        decel_power: 0.7,
    };

    let robot = RobotState {
        x: -3.0,
        y: -3.0,
        theta: 0.0,
    };

    let goal = pt2(3.0, 3.0);

    let path = Vec::new();

    Model {
        robot,
        params,
        goal,
        path,
        reached_goal: false,
    }
}

impl Model {
    // use &self to reference the struct's data without taking ownership
    fn distance_to_goal(&self) -> f32 {
        let dx = self.goal.x - self.robot.x;
        let dy = self.goal.y - self.robot.y;
        (dx * dx + dy * dy).sqrt()
    }

    fn vector_field(&self, x: f32, y: f32) -> (f32, f32) {
        let dx = self.goal.x - x;
        let dy = self.goal.y - y;
        let magnitude = (dx * dx + dy * dy).sqrt();
        
        if magnitude > 0.0 {
            (dx / magnitude, dy / magnitude)
        } else {
            (0.0, 0.0)
        }
    }

    fn compute_velocity_scaling(&self) -> f32 {
        let distance = self.distance_to_goal();
        
        if distance < self.params.stop_threshold {
            0.0
        } else if distance < self.params.goal_threshold {
            self.params.min_velocity / self.params.v_max
        } else if distance < self.params.decel_radius {
            let scale = ((distance - self.params.goal_threshold) /
                (self.params.decel_radius - self.params.goal_threshold))
                .powf(self.params.decel_power);
            scale.max(self.params.min_velocity / self.params.v_max)
        } else {
            1.0
        }
    }

    fn compute_control(&self) -> (f32, f32) {
        if self.reached_goal {
            return (0.0, 0.0);
        }

        let (dx, dy) = self.vector_field(self.robot.x, self.robot.y);
        let desired_theta = dy.atan2(dx);
        
        let theta_error = (desired_theta - self.robot.theta)
            .rem_euclid(2.0 * PI);
        let wrapped_error = if theta_error > PI {
            theta_error - 2.0 * PI
        } else {
            theta_error
        };

        let velocity_scale = self.compute_velocity_scaling();
        let v = self.params.v_max * velocity_scale;
        let omega = self.params.k_p * wrapped_error;

        (v, omega)
    }
}

// update frame
fn update(_app: &App, model: &mut Model, _update: Update) {
    if model.reached_goal {
        return;
    }

    // update control inputs
    let (v, omega) = model.compute_control();
    
    // update robot state
    let dt = 1.0 / 60.0; // 60 fps
    model.robot.x += v * model.robot.theta.cos() * dt;
    model.robot.y += v * model.robot.theta.sin() * dt;
    model.robot.theta += omega * dt;

    // update robot path
    model.path.push(pt2(model.robot.x, model.robot.y));

    if model.distance_to_goal() < model.params.stop_threshold {
        model.reached_goal = true;
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    
    draw.background().color(WHITE);

    // setup sim coords
    let scaling = 100.0;
    
    // draw vf
    let grid_resolution = 20;
    let field_size = 8.0;
    let step = field_size / grid_resolution as f32;
    
    for i in 0..grid_resolution {
        for j in 0..grid_resolution {
            let x = -field_size/2.0 + i as f32 * step;
            let y = -field_size/2.0 + j as f32 * step;
            
            let (dx, dy) = model.vector_field(x, y);
            
            let arrow_length = 0.3;
            let start = pt2(x, y) * scaling;
            let end = pt2(x + dx * arrow_length, y + dy * arrow_length) * scaling;
            
            draw.line()
                .start(start)
                .end(end)
                .stroke_weight(1.0)
                .color(rgba(0.5, 0.5, 0.5, 0.5));
            
            let arrow_head_length = 0.1 * scaling;
            let angle = dy.atan2(dx);
            let head_angle = PI / 6.0; // 30 degrees
            
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
                .color(rgba(0.5, 0.5, 0.5, 0.5));
            
            draw.line()
                .start(end)
                .end(head2)
                .stroke_weight(1.0)
                .color(rgba(0.5, 0.5, 0.5, 0.5));
        }
    }
    
    // decel radius
    draw.ellipse()
        .xy(model.goal * scaling)
        .radius(model.params.decel_radius * scaling)
        .color(rgba(1.0, 1.0, 0.0, 0.1));

    // goal point
    draw.ellipse()
        .xy(model.goal * scaling)
        .radius(model.params.goal_threshold * scaling)
        .color(rgba(0.0, 1.0, 0.0, 0.3));

    // robot path (updates)
    if model.path.len() >= 2 {
        draw.polyline()
            .points(model.path.iter().map(|p| *p * scaling))
            .color(BLUE);
    }

    // robot
    let robot_pos = pt2(model.robot.x, model.robot.y) * scaling;
    draw.ellipse()
        .xy(robot_pos)
        .radius(10.0)
        .color(BLUE);
    
    // robot heading
    let heading_end = robot_pos + pt2(
        model.robot.theta.cos() * 20.0,
        model.robot.theta.sin() * 20.0
    );
    draw.line()
        .start(robot_pos)
        .end(heading_end)
        .color(RED);

    draw.to_frame(app, &frame).unwrap();
}