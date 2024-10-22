use nannou::prelude::*;
use std::f32::consts::PI;
use crate::types::Model;

impl Model {
    pub fn evaluate_bezier_curve(&self, t: f32) -> Point2 {
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
    
    pub fn generate_bezier_path(&mut self, num_points: usize) {
        self.path = (0..num_points).map(|i| {
            let t = i as f32 / (num_points - 1) as f32;
            self.evaluate_bezier_curve(t)
        }).collect();
    }

    pub fn vf_distance_to_goal(&self) -> f32 {
        let dx = self.goal.x - self.vector_field_robot.x;
        let dy = self.goal.y - self.vector_field_robot.y;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn pp_distance_to_goal(&self) -> f32 {
        let dx = self.goal.x - self.pure_pursuit_robot.x;
        let dy = self.goal.y - self.pure_pursuit_robot.y;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn find_closest_point_on_path(&self, point: Point2) -> (f32, f32) {
        (0..=100).map(|i| {
            let t = i as f32 / 100.0;
            let path_point = self.evaluate_bezier_curve(t);
            let distance = (path_point - point).length();
            (t, distance)
        }).min_by(|a, b| a.1.partial_cmp(&b.1).unwrap()).unwrap()
    }

    pub fn calculate_path_tangent(&self, t: f32) -> Vec2 {
        if t < 0.999 {
            (self.evaluate_bezier_curve(t + 0.001) - self.evaluate_bezier_curve(t)).normalize()
        } else {
            (self.evaluate_bezier_curve(1.0) - self.evaluate_bezier_curve(0.999)).normalize()
        }
    }

    pub fn calculate_target_point(&self, point: Point2) -> Point2 {
        let (t, _) = self.find_closest_point_on_path(point);
        let path_point = self.evaluate_bezier_curve(t);
        let tangent = self.calculate_path_tangent(t);
        path_point + tangent
    }

    pub fn calculate_vector_field(&self, x: f32, y: f32) -> (f32, f32) {
        let point = pt2(x, y);
        let target = self.calculate_target_point(point);
        let (_, distance) = self.find_closest_point_on_path(point);
        let path_direction = (target - point).normalize();
        let mut direction = path_direction;
    
        for obstacle in &self.obstacles {
            let obstacle_vector = point - obstacle.position;
            let obstacle_distance = obstacle_vector.length();
            let obstacle_influence_radius = obstacle.radius * 5.0;
    
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

    pub fn calculate_velocity_scaling(&self) -> f32 {
        let distance = self.vf_distance_to_goal();
        
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

    pub fn compute_control(&self) -> (f32, f32) {
        if self.vf_reached_goal {
            return (0.0, 0.0);
        }
    
        let distance_to_goal = self.vf_distance_to_goal();
        let (dx, dy) = if distance_to_goal <= self.params.decel_radius {
            (self.goal.x - self.vector_field_robot.x, self.goal.y - self.vector_field_robot.y)
        } else {
            self.calculate_vector_field(self.vector_field_robot.x, self.vector_field_robot.y)
        };
    
        let desired_theta = dy.atan2(dx);
        
        let theta_error = (desired_theta - self.vector_field_robot.theta)
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

    pub fn find_lookahead_point(&self, robot_pos: Point2) -> Point2 {
        let mut closest_idx = 0;
        let mut min_dist = f32::MAX;
        
        for (i, path_point) in self.path.iter().enumerate() {
            let dist = (*path_point - robot_pos).length();
            if dist < min_dist {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        let lookahead_dist = self.params.pure_pursuit_lookahead;
        let mut accumulated_dist = 0.0;
        let mut target_point = self.path[closest_idx];
        
        for i in closest_idx..self.path.len()-1 {
            let segment_dist = (self.path[i+1] - self.path[i]).length();
            accumulated_dist += segment_dist;
            
            if accumulated_dist >= lookahead_dist {
                target_point = self.path[i+1];
                break;
            }
        }
        
        target_point
    }

    pub fn compute_pure_pursuit_control(&self) -> (f32, f32) {
        if self.pp_reached_goal {
            return (0.0, 0.0)
        }
        
        let robot_pos = pt2(self.pure_pursuit_robot.x, self.pure_pursuit_robot.y);
        let target = self.find_lookahead_point(robot_pos);
        
        let dx = target.x - self.pure_pursuit_robot.x;
        let dy = target.y - self.pure_pursuit_robot.y;
        let target_distance = (dx * dx + dy * dy).sqrt();
        
        let desired_theta = dy.atan2(dx);
        let theta_error = (desired_theta - self.pure_pursuit_robot.theta).rem_euclid(2.0 * PI);
        let wrapped_error = if theta_error > PI {
            theta_error - 2.0 * PI
        } else {
            theta_error
        };
        
        let goal_distance = ((self.goal.x - self.pure_pursuit_robot.x).powi(2) + 
                           (self.goal.y - self.pure_pursuit_robot.y).powi(2)).sqrt();
        
        let velocity_scale = if goal_distance < self.params.stop_threshold {
            0.0
        } else if goal_distance < self.params.goal_threshold {
            self.params.min_velocity / self.params.v_max
        } else if goal_distance < self.params.decel_radius {
            (goal_distance / self.params.decel_radius)
                .max(self.params.min_velocity / self.params.v_max)
        } else {
            1.0
        };
        
        let v = self.params.v_max * velocity_scale;
        let omega = (2.0 * v * wrapped_error / target_distance).clamp(-4.0, 4.0);
        
        (v, omega)
    }
}