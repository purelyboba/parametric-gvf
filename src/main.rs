use nannou::prelude::*;
use std::f32::consts::PI;

struct RobotState {
    x: f32,
    y: f32,
    theta: f32,
}

struct SimulationParams {
    v_max: f32,
    k_p: f32,
    decel_radius: f32,
    goal_threshold: f32,
    stop_threshold: f32,
    min_velocity: f32,
}

struct Obstacle {
    position: Point2,
    radius: f32,
}

struct Model {
    robot: RobotState,
    params: SimulationParams,
    goal: Point2,  
    path: Vec<Point2>,
    path_control_points: [Point2; 4],
    followed_path: Vec<Point2>,
    reached_goal: bool,
    simulation_time: f32,
    obstacles: Vec<Obstacle>,
}

fn main() {
    nannou::app(model)
        .update(update)
        .run();
}

fn model(app: &App) -> Model {
    app.new_window().size(800, 800).view(view).build().unwrap();
    
    let params = SimulationParams {
        v_max: 3.0,
        k_p: 20.0,
        decel_radius: 1.0,
        goal_threshold: 0.1,
        stop_threshold: 0.01,
        min_velocity: 0.1,
    };

    let robot = RobotState {
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

    let mut obstacles = Vec::new();
    obstacles.push(Obstacle {
        position: pt2(-3.0, 3.0),
        radius: 0.2,
    });
    obstacles.push(Obstacle {
        position: pt2(3.0, -3.0),
        radius: 0.2,
    });
    obstacles.push(Obstacle {
        position: pt2(0.0, 0.0),
        radius: 0.2,
    });
    obstacles.push(Obstacle {
        position: pt2(0.0, 3.0),
        radius: 0.2,
    });     
    obstacles.push(Obstacle {
        position: pt2(0.0, -3.0),
        radius: 0.2,
    });
    obstacles.push(Obstacle {
        position: pt2(3.0, 0.0),
        radius: 0.2,
    });
    obstacles.push(Obstacle {
        position: pt2(-3.0, 0.0),
        radius: 0.2,
    });
    obstacles.push(Obstacle {
        position: pt2(-3.0, -3.0),
        radius: 0.2,
    });
    obstacles.push(Obstacle {
        position: pt2(3.0, 3.0),
        radius: 0.2,
    });

    let mut model = Model {
        robot,
        params,
        goal,
        followed_path: Vec::new(),
        reached_goal: false,
        path: Vec::new(),
        path_control_points,
        simulation_time: 0.0,
        obstacles,
    };

    model.generate_bezier_path(100);

    model
}

impl Model {
    fn evaluate_bezier_curve(&self, t: f32) -> Point2 {
        let [p0, p1, p2, p3] = self.path_control_points;
        let one_minus_t = 1.0 - t;
        let t_squared = t * t;
        let one_minus_t_squared = one_minus_t * one_minus_t;
        let t_cubed = t_squared * t;
        let one_minus_t_cubed = one_minus_t_squared * one_minus_t;

        p0 * one_minus_t_cubed +
        p1 * 3.0 * one_minus_t_squared * t +
        p2 * 3.0 * one_minus_t * t_squared +
        p3 * t_cubed
    }
    
    fn generate_bezier_path(&mut self, num_points: usize) {
        self.path = (0..num_points).map(|i| {
            let t = i as f32 / (num_points - 1) as f32;
            self.evaluate_bezier_curve(t)
        }).collect();
    }

    fn distance_to_goal(&self) -> f32 {
        let dx = self.goal.x - self.robot.x;
        let dy = self.goal.y - self.robot.y;
        (dx * dx + dy * dy).sqrt()
    }

    fn find_closest_point_on_path(&self, point: Point2) -> (f32, f32) {
        (0..=100).map(|i| {
            let t = i as f32 / 100.0;
            let path_point = self.evaluate_bezier_curve(t);
            let distance = (path_point - point).length();
            (t, distance)
        }).min_by(|a, b| a.1.partial_cmp(&b.1).unwrap()).unwrap()
    }

    fn calculate_path_tangent(&self, t: f32) -> Vec2 {
        if t < 0.999 {
            (self.evaluate_bezier_curve(t + 0.001) - self.evaluate_bezier_curve(t)).normalize()
        } else {
            (self.evaluate_bezier_curve(1.0) - self.evaluate_bezier_curve(0.999)).normalize()
        }
    }

    fn calculate_target_point(&self, point: Point2) -> Point2 {
        let (t, _) = self.find_closest_point_on_path(point);
        let path_point = self.evaluate_bezier_curve(t);
        let tangent = self.calculate_path_tangent(t);
        path_point + tangent
    }

    fn calculate_vector_field(&self, x: f32, y: f32) -> (f32, f32) {
        let point = pt2(x, y);
        let target = self.calculate_target_point(point);
        let (_, distance) = self.find_closest_point_on_path(point);
        let path_direction = (target - point).normalize();
        let mut direction = path_direction;
    
        for obstacle in &self.obstacles {
            let obstacle_vector = point - obstacle.position;
            let obstacle_distance = obstacle_vector.length();
            let obstacle_influence_radius = obstacle.radius * 5.0; // Increased influence radius
    
            if obstacle_distance < obstacle_influence_radius {
                let influence_strength = 1.0 - (obstacle_distance / obstacle_influence_radius).powi(2);
                let obstacle_influence = influence_strength;
                let obstacle_direction = obstacle_vector.normalize();
                let perpendicular = vec2(-path_direction.y, path_direction.x);
                let side = perpendicular.dot(obstacle_vector).signum();
                let avoidance_vector = (obstacle_direction + side * perpendicular).normalize();
                direction = (path_direction + avoidance_vector * obstacle_influence).normalize();
            }
        }
    
        if !direction.x.is_finite() || !direction.y.is_finite() || !distance.is_finite() {
            return (0.0, 0.0);
        }
    
        (direction.x * distance, direction.y * distance)
    }

    fn calculate_velocity_scaling(&self) -> f32 {
        let distance = self.distance_to_goal();
        
        if distance < self.params.stop_threshold {
            0.0
        } else if distance < self.params.goal_threshold {
            self.params.min_velocity / self.params.v_max
        } else if distance < self.params.decel_radius {
            (distance / self.params.decel_radius).max(self.params.min_velocity / self.params.v_max)
        } else {
            1.0
        }
    }

    fn compute_control(&self) -> (f32, f32) {
        if self.reached_goal {
            return (0.0, 0.0);
        }
    
        let distance_to_goal = self.distance_to_goal();
        let (dx, dy) = if distance_to_goal <= self.params.decel_radius {
            (self.goal.x - self.robot.x, self.goal.y - self.robot.y)
        } else {
            self.calculate_vector_field(self.robot.x, self.robot.y)
        };
    
        let desired_theta = dy.atan2(dx);
        
        let theta_error = (desired_theta - self.robot.theta)
            .rem_euclid(2.0 * PI);
        let wrapped_error = if theta_error > PI {
            theta_error - 2.0 * PI
        } else {
            theta_error
        };
    
        let velocity_scale = self.calculate_velocity_scaling();
        let v = self.params.v_max * velocity_scale;
        let omega = (self.params.k_p * wrapped_error).clamp(-4.0, 4.0);
    
        (v, omega)
    }    
}

fn update(_app: &App, model: &mut Model, update: Update) {
    if model.reached_goal {
        return;
    }

    let (v, omega) = model.compute_control();
    
    let dt = update.since_last.as_secs_f32();
    model.robot.x += v * model.robot.theta.cos() * dt;
    model.robot.y += v * model.robot.theta.sin() * dt;
    model.robot.theta += omega * dt;

    model.followed_path.push(pt2(model.robot.x, model.robot.y));

    model.simulation_time += dt;

    let distance_to_goal = model.distance_to_goal();
    if distance_to_goal < model.params.stop_threshold {
        model.reached_goal = true;
        // Snap to exact goal position
        model.robot.x = model.goal.x;
        model.robot.y = model.goal.y;
        model.followed_path.push(model.goal);
        println!("Goal reached in {:.2} seconds!", model.simulation_time);
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
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
    
    draw.ellipse()
        .xy(model.goal * scaling)
        .radius(model.params.decel_radius * scaling)
        .color(rgba(1.0, 1.0, 0.0, 0.1));

    draw.ellipse()
        .xy(model.goal * scaling)
        .radius(model.params.goal_threshold * scaling)
        .color(rgba(0.0, 1.0, 0.0, 0.3));

    if model.followed_path.len() >= 2 {
        draw.polyline()
            .points(model.followed_path.iter().map(|p| *p * scaling))
            .color(BLUE);
    }

    if model.path.len() >= 2 {
        draw.polyline()
            .points(model.path.iter().map(|p| *p * scaling))
            .color(rgba(1.0, 0.0, 1.0, 0.5));
    }

    for obstacle in &model.obstacles {
        draw.ellipse()
            .xy(obstacle.position * scaling)
            .radius(obstacle.radius * scaling)
            .color(rgba(1.0, 0.0, 0.0, 0.5));
    }

    let robot_pos = pt2(model.robot.x, model.robot.y) * scaling;
    draw.ellipse()
        .xy(robot_pos)
        .radius(10.0)
        .color(BLUE);
    
    let heading_end = robot_pos + pt2(
        model.robot.theta.cos() * 20.0,
        model.robot.theta.sin() * 20.0
    );
    draw.line()
        .start(robot_pos)
        .end(heading_end)
        .color(RED);

    let time_text = format!("Time: {:.2}s", model.simulation_time);
    draw.text(&time_text)
        .xy(pt2(-350.0, 350.0))
        .color(BLACK)
        .font_size(20);

    if model.reached_goal {
        draw.text("Goal reached")
            .xy(pt2(-350.0, 320.0))
            .color(BLACK)
            .font_size(20);
    }

    draw.to_frame(app, &frame).unwrap();
}