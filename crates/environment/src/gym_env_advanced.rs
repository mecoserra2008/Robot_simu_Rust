use crate::{DomainRandomizer, PhysicsObject, ObjectShape, MaterialType, GoalManager, GoalType};
use robot::{QuadrupedRobot, RobotConfig};
use physics::{PhysicsEngine, na, vector};
use rand::{Rng, SeedableRng};

// Enhanced observation space: 51 + 10 (head/grippers) + 20 (goals) = 81
pub const OBS_DIM: usize = 81;
// Enhanced action space: 12 (legs) + 2 (head) + 4 (grippers) = 18
pub const ACT_DIM: usize = 18;

pub struct AdvancedRobotEnv {
    pub physics: PhysicsEngine,
    pub robot: QuadrupedRobot,
    pub objects: Vec<PhysicsObject>,
    pub goal_manager: GoalManager,
    pub randomizer: DomainRandomizer,
    pub step_count: usize,
    pub max_steps: usize,
    pub episode_reward: f32,
    pub prev_action: Vec<f32>,
    pub rng: rand::rngs::StdRng,
    pub simulation_time: f32,
}

impl AdvancedRobotEnv {
    pub fn new(robot_config: RobotConfig) -> Self {
        let mut physics = PhysicsEngine::new(0.002, na::Vector3::new(0.0, -9.81, 0.0));
        let robot = QuadrupedRobot::new(&mut physics, &robot_config);

        // Create diverse objects
        let objects = Self::create_diverse_objects(&mut physics);

        let mut goal_manager = GoalManager::new();
        // Add initial exploration goal
        goal_manager.add_goal(GoalType::Explore {
            region: na::Vector3::new(5.0, 0.0, 0.0),
            curiosity: 1.0,
        });

        Self {
            physics,
            robot,
            objects,
            goal_manager,
            randomizer: DomainRandomizer::default(),
            step_count: 0,
            max_steps: 2000,
            episode_reward: 0.0,
            prev_action: vec![0.0; ACT_DIM],
            rng: rand::rngs::StdRng::from_entropy(),
            simulation_time: 0.0,
        }
    }

    fn create_diverse_objects(physics: &mut PhysicsEngine) -> Vec<PhysicsObject> {
        let mut objects = Vec::new();

        // Wooden box
        objects.push(PhysicsObject::new(
            physics,
            "wooden_crate".to_string(),
            ObjectShape::Box { dimensions: [0.5, 0.5, 0.5] },
            MaterialType::Wood,
            false,
        ));

        // Metal sphere (heavy, dangerous when falling)
        objects.push(PhysicsObject::new(
            physics,
            "metal_ball".to_string(),
            ObjectShape::Sphere { radius: 0.2 },
            MaterialType::Metal,
            false,
        ));

        // Glass vase (fragile)
        objects.push(PhysicsObject::new(
            physics,
            "glass_vase".to_string(),
            ObjectShape::Cylinder { height: 0.4, radius: 0.15 },
            MaterialType::Glass,
            false,
        ));

        // Rubber ball (bouncy)
        objects.push(PhysicsObject::new(
            physics,
            "rubber_ball".to_string(),
            ObjectShape::Sphere { radius: 0.15 },
            MaterialType::Rubber,
            false,
        ));

        // Plastic container
        objects.push(PhysicsObject::new(
            physics,
            "plastic_container".to_string(),
            ObjectShape::Box { dimensions: [0.3, 0.4, 0.3] },
            MaterialType::Plastic,
            false,
        ));

        // Stone pillar (very heavy)
        objects.push(PhysicsObject::new(
            physics,
            "stone_pillar".to_string(),
            ObjectShape::Cylinder { height: 1.0, radius: 0.2 },
            MaterialType::Stone,
            false,
        ));

        // Metal cone (pointy, dangerous)
        objects.push(PhysicsObject::new(
            physics,
            "metal_cone".to_string(),
            ObjectShape::Cone { height: 0.6, radius: 0.25 },
            MaterialType::Metal,
            false,
        ));

        // Wooden capsule
        objects.push(PhysicsObject::new(
            physics,
            "wooden_log".to_string(),
            ObjectShape::Capsule { height: 0.8, radius: 0.1 },
            MaterialType::Wood,
            false,
        ));

        objects
    }

    pub fn reset(&mut self) -> Vec<f32> {
        self.randomizer.randomize_physics(&mut self.physics, &mut self.rng);

        // Reset robot
        if let Some(body) = self.physics.get_body_mut(self.robot.base_body) {
            body.set_translation(vector![0.0, 0.5, 0.0], true);
            body.set_linvel(vector![0.0, 0.0, 0.0], true);
            body.set_angvel(vector![0.0, 0.0, 0.0], true);
            body.set_rotation(na::UnitQuaternion::identity(), true);
        }

        // Reset robot health
        self.robot.health = self.robot.max_health;

        // Reset objects
        for obj in &mut self.objects {
            obj.health = obj.max_health;
            obj.is_dangerous = false;
        }

        // Reset goals
        self.goal_manager.clear_goals();

        // Add initial navigation goal
        let angle = self.rng.gen_range(0.0..std::f32::consts::TAU);
        let distance = self.rng.gen_range(3.0..8.0);
        self.goal_manager.add_goal(GoalType::Navigate {
            target: na::Vector3::new(distance * angle.cos(), 0.0, distance * angle.sin()),
            urgency: 0.5,
        });

        self.step_count = 0;
        self.episode_reward = 0.0;
        self.prev_action = vec![0.0; ACT_DIM];
        self.simulation_time = 0.0;

        self.get_observation()
    }

    pub fn step(&mut self, action: &[f32]) -> (Vec<f32>, f32, bool, AdvancedStepInfo) {
        assert_eq!(action.len(), ACT_DIM);

        // Apply actions to robot
        self.apply_action(action);

        // Update actuators
        let substeps = 10;
        let dt_substep = self.physics.dt / substeps as f32;
        for _ in 0..substeps {
            self.robot.update_actuators(dt_substep);
        }

        // Step physics
        self.physics.step();
        self.simulation_time += self.physics.dt;

        // Update objects
        for obj in &mut self.objects {
            obj.update(&self.physics, self.physics.dt);
        }

        // Update goal manager
        self.goal_manager.update(self.physics.dt);

        // Detect threats and add avoidance goals
        self.detect_and_respond_to_threats();

        // Check for damage
        self.check_collisions();

        // Compute reward
        let (reward, info) = self.compute_advanced_reward(action);

        // Check termination
        let done = self.check_done(&info);

        self.step_count += 1;
        self.episode_reward += reward;
        self.prev_action = action.to_vec();

        let obs = self.get_observation();
        (obs, reward, done, info)
    }

    fn apply_action(&mut self, action: &[f32]) {
        // Leg actions (12)
        for (i, leg) in self.robot.legs.iter_mut().enumerate() {
            let base = i * 3;
            leg.hip_x.set_position_target(action[base]);
            leg.hip_y.set_position_target(action[base + 1]);
            leg.knee.set_position_target(action[base + 2]);
        }

        // Head actions (2)
        self.robot.head.pitch.set_position_target(action[12]);
        self.robot.head.yaw.set_position_target(action[13]);

        // Gripper actions (4 - one per leg)
        for (i, leg) in self.robot.legs.iter_mut().enumerate() {
            leg.gripper.open_close.set_position_target(action[14 + i]);
        }
    }

    fn detect_and_respond_to_threats(&mut self) {
        if let Some(robot_body) = self.physics.get_body(self.robot.base_body) {
            let robot_pos = na::Vector3::new(
                robot_body.translation().x,
                robot_body.translation().y,
                robot_body.translation().z,
            );

            for obj in &self.objects {
                if obj.is_dangerous {
                    if let Some(obj_body) = self.physics.get_body(obj.body_handle) {
                        let obj_pos = na::Vector3::new(
                            obj_body.translation().x,
                            obj_body.translation().y,
                            obj_body.translation().z,
                        );
                        let obj_vel = na::Vector3::new(
                            obj_body.linvel().x,
                            obj_body.linvel().y,
                            obj_body.linvel().z,
                        );

                        let distance = (obj_pos - robot_pos).norm();

                        // If dangerous object is close, add high-priority avoidance goal
                        if distance < 3.0 {
                            let urgency = (3.0 - distance) / 3.0;
                            self.goal_manager.add_goal(GoalType::AvoidThreat {
                                threat_pos: obj_pos,
                                threat_velocity: obj_vel,
                                urgency,
                            });
                        }
                    }
                }
            }

            // Check health and add survival/heal goals
            let health_ratio = self.robot.health / self.robot.max_health;
            if health_ratio < 0.3 {
                self.goal_manager.add_goal(GoalType::Survive {
                    health_threshold: 30.0,
                    urgency: 1.0 - health_ratio,
                });
            } else if health_ratio < 0.7 {
                self.goal_manager.add_goal(GoalType::RestoreHealth {
                    urgency: 0.7 - health_ratio,
                });
            }
        }
    }

    fn check_collisions(&mut self) {
        // Check robot collisions with dangerous objects
        if let Some(robot_body) = self.physics.get_body(self.robot.base_body) {
            let robot_pos = na::Vector3::new(
                robot_body.translation().x,
                robot_body.translation().y,
                robot_body.translation().z,
            );

            for obj in &self.objects {
                if obj.is_dangerous {
                    if let Some(obj_body) = self.physics.get_body(obj.body_handle) {
                        let obj_pos = na::Vector3::new(
                            obj_body.translation().x,
                            obj_body.translation().y,
                            obj_body.translation().z,
                        );

                        let distance = (obj_pos - robot_pos).norm();
                        if distance < 0.5 {
                            // Collision detected!
                            self.robot.health -= obj.damage_on_contact;
                        }
                    }
                }
            }
        }
    }

    fn get_observation(&self) -> Vec<f32> {
        let mut obs = Vec::with_capacity(OBS_DIM);

        // Base robot observation (48)
        let base_obs = self.robot.get_observation(&self.physics, &mut self.rng.clone());
        obs.extend_from_slice(&base_obs);

        // Previous actions (18)
        obs.extend_from_slice(&self.prev_action);

        // Health status (1)
        obs.push(self.robot.health / self.robot.max_health);

        // Current goals (top 3) (12 = 3 goals × 4 values each)
        let top_goals = self.goal_manager.get_top_goals(3);
        for goal in top_goals {
            obs.push(goal.goal_type.get_priority() / 100.0);
            obs.push(goal.goal_type.get_expected_reward() / 1000.0);
            obs.push(match goal.deadline {
                Some(d) => (d - self.simulation_time).max(0.0) / 10.0,
                None => 1.0,
            });
            obs.push(if goal.active { 1.0 } else { 0.0 });
        }
        // Pad if fewer than 3 goals
        while obs.len() < 48 + 18 + 1 + 12 {
            obs.push(0.0);
        }

        // Nearby threats (2 = closest threat distance + velocity)
        let mut min_threat_dist = 100.0;
        let mut threat_velocity = 0.0;
        if let Some(robot_body) = self.physics.get_body(self.robot.base_body) {
            let robot_pos = na::Vector3::new(
                robot_body.translation().x,
                robot_body.translation().y,
                robot_body.translation().z,
            );

            for obj in &self.objects {
                if obj.is_dangerous {
                    if let Some(obj_body) = self.physics.get_body(obj.body_handle) {
                        let obj_pos = na::Vector3::new(
                            obj_body.translation().x,
                            obj_body.translation().y,
                            obj_body.translation().z,
                        );
                        let distance = (obj_pos - robot_pos).norm();
                        if distance < min_threat_dist {
                            min_threat_dist = distance;
                            threat_velocity = na::Vector3::new(
                                obj_body.linvel().x,
                                obj_body.linvel().y,
                                obj_body.linvel().z,
                            ).norm();
                        }
                    }
                }
            }
        }
        obs.push((min_threat_dist / 10.0).min(1.0));
        obs.push((threat_velocity / 10.0).min(1.0));

        obs
    }

    fn compute_advanced_reward(&self, action: &[f32]) -> (f32, AdvancedStepInfo) {
        let mut reward = 0.0;
        let mut info = AdvancedStepInfo::default();

        if let Some(body) = self.physics.get_body(self.robot.base_body) {
            let pos = body.translation();
            let vel = body.linvel();
            let rot = body.rotation();

            // Goal-based rewards
            if let Some(current_goal) = self.goal_manager.get_current_goal() {
                match &current_goal.goal_type {
                    GoalType::Navigate { target, .. } => {
                        let to_target = target - na::Vector3::new(pos.x, pos.y, pos.z);
                        let target_dir = to_target.normalize();
                        let velocity_proj = vel.x * target_dir.x + vel.z * target_dir.z;
                        reward += velocity_proj.max(0.0);

                        let distance = to_target.norm();
                        if distance < 0.5 {
                            reward += 100.0;
                            info.goal_completed = true;
                        }
                    }
                    GoalType::AvoidThreat { threat_pos, .. } => {
                        let to_threat = threat_pos - na::Vector3::new(pos.x, pos.y, pos.z);
                        let distance = to_threat.norm();
                        reward += (distance - 1.0).max(0.0) * 10.0;
                    }
                    _ => {}
                }
            }

            // Survival rewards
            let health_ratio = self.robot.health / self.robot.max_health;
            reward += health_ratio * 5.0;
            info.health = self.robot.health;

            // Stability
            let (roll, pitch, _) = rot.euler_angles();
            let r_stability = -((roll.powi(2) + pitch.powi(2)) * 2.0);
            reward += r_stability * 0.5;

            // Energy efficiency
            let mut power = 0.0;
            for leg in &self.robot.legs {
                power += (leg.hip_x.state.torque * leg.hip_x.state.velocity).abs();
                power += (leg.hip_y.state.torque * leg.hip_y.state.velocity).abs();
                power += (leg.knee.state.torque * leg.knee.state.velocity).abs();
            }
            reward += -power / 500.0 * 0.05;

            // Action smoothness
            let mut action_diff = 0.0;
            for i in 0..action.len().min(self.prev_action.len()) {
                action_diff += (action[i] - self.prev_action[i]).powi(2);
            }
            reward += -action_diff * 0.1;

            // Penalties
            if roll.abs() > 1.0 || pitch.abs() > 1.0 || pos.y < 0.1 {
                reward -= 100.0;
                info.termination = Some("fallen".to_string());
            }

            if self.robot.health <= 0.0 {
                reward -= 200.0;
                info.termination = Some("destroyed".to_string());
            }
        }

        (reward, info)
    }

    fn check_done(&self, info: &AdvancedStepInfo) -> bool {
        info.termination.is_some() || self.step_count >= self.max_steps || self.robot.health <= 0.0
    }
}

#[derive(Debug, Clone, Default)]
pub struct AdvancedStepInfo {
    pub health: f32,
    pub goal_completed: bool,
    pub objects_manipulated: usize,
    pub threats_avoided: usize,
    pub termination: Option<String>,
}
