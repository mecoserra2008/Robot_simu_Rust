use crate::{World, DomainRandomizer, ObjectConfig};
use robot::{QuadrupedRobot, RobotConfig};
use physics::{PhysicsEngine, na, vector};
use rand::{Rng, SeedableRng};

pub const OBS_DIM: usize = 51;
pub const ACT_DIM: usize = 12;

pub struct RobotEnv {
    pub physics: PhysicsEngine,
    pub robot: QuadrupedRobot,
    pub world: World,
    pub randomizer: DomainRandomizer,
    pub step_count: usize,
    pub max_steps: usize,
    pub episode_reward: f32,
    pub prev_action: Vec<f32>,
    pub target_position: na::Vector3<f32>,
    pub rng: rand::rngs::StdRng,
}

impl RobotEnv {
    pub fn new(robot_config: RobotConfig, object_configs: Vec<ObjectConfig>) -> Self {
        let mut physics = PhysicsEngine::new(0.002, na::Vector3::new(0.0, -9.81, 0.0));
        let robot = QuadrupedRobot::new(&mut physics, &robot_config);
        let world = World::new(&mut physics, object_configs);

        Self {
            physics,
            robot,
            world,
            randomizer: DomainRandomizer::default(),
            step_count: 0,
            max_steps: 1000,
            episode_reward: 0.0,
            prev_action: vec![0.0; ACT_DIM],
            target_position: na::Vector3::new(5.0, 0.0, 0.0),
            rng: rand::rngs::StdRng::from_entropy(),
        }
    }

    pub fn reset(&mut self) -> Vec<f32> {
        self.randomizer.randomize_physics(&mut self.physics, &mut self.rng);

        if let Some(body) = self.physics.get_body_mut(self.robot.base_body) {
            body.set_translation(vector![0.0, 0.5, 0.0], true);
            body.set_linvel(vector![0.0, 0.0, 0.0], true);
            body.set_angvel(vector![0.0, 0.0, 0.0], true);
            body.set_rotation(na::UnitQuaternion::identity(), true);
        }

        let angle = self.rng.gen_range(0.0..std::f32::consts::TAU);
        let distance = self.rng.gen_range(3.0..8.0);
        self.target_position = na::Vector3::new(distance * angle.cos(), 0.0, distance * angle.sin());

        self.step_count = 0;
        self.episode_reward = 0.0;
        self.prev_action = vec![0.0; ACT_DIM];

        self.get_observation()
    }

    pub fn step(&mut self, action: &[f32]) -> (Vec<f32>, f32, bool, StepInfo) {
        assert_eq!(action.len(), ACT_DIM);

        self.robot.apply_action(action);

        let substeps = 10;
        for _ in 0..substeps {
            self.robot.update_actuators(self.physics.dt / substeps as f32);
        }

        self.physics.step();
        let dt = self.physics.dt;
        self.world.update(&mut self.physics, dt);

        let (reward, info) = self.compute_reward(action);
        let done = self.check_done(&info);

        self.step_count += 1;
        self.episode_reward += reward;
        self.prev_action = action.to_vec();

        let obs = self.get_observation();
        (obs, reward, done, info)
    }

    fn get_observation(&self) -> Vec<f32> {
        let mut obs = self.robot.get_observation(&self.physics, &mut self.rng.clone());
        obs.extend_from_slice(&self.prev_action);

        if let Some(body) = self.physics.get_body(self.robot.base_body) {
            let pos = body.translation();
            let to_target = self.target_position - na::Vector3::new(pos.x, pos.y, pos.z);
            obs.push(to_target.x);
            obs.push(to_target.z);
            obs.push(to_target.norm());
        }

        obs
    }

    fn compute_reward(&self, action: &[f32]) -> (f32, StepInfo) {
        let mut reward = 0.0;
        let mut info = StepInfo::default();

        if let Some(body) = self.physics.get_body(self.robot.base_body) {
            let pos = body.translation();
            let vel = body.linvel();
            let rot = body.rotation();

            let to_target = self.target_position - na::Vector3::new(pos.x, pos.y, pos.z);
            let target_dir = to_target.normalize();
            let velocity_proj = vel.x * target_dir.x + vel.z * target_dir.z;
            let r_velocity = velocity_proj.max(0.0) - (velocity_proj - 1.0).abs();
            reward += r_velocity;
            info.r_velocity = r_velocity;

            let distance = to_target.norm();
            let r_distance = -distance * 0.1;
            reward += r_distance;
            info.r_distance = r_distance;

            let (roll, pitch, _) = rot.euler_angles();
            let r_stability = -((roll.powi(2) + pitch.powi(2)) * 2.0);
            reward += r_stability * 0.5;
            info.r_stability = r_stability;

            let mut power = 0.0;
            for leg in &self.robot.legs {
                power += (leg.hip_x.state.torque * leg.hip_x.state.velocity).abs();
                power += (leg.hip_y.state.torque * leg.hip_y.state.velocity).abs();
                power += (leg.knee.state.torque * leg.knee.state.velocity).abs();
            }
            let r_energy = -power / 500.0;
            reward += r_energy * 0.05;
            info.r_energy = r_energy;

            let mut action_diff = 0.0;
            for i in 0..action.len() {
                action_diff += (action[i] - self.prev_action[i]).powi(2);
            }
            let r_smoothness = -action_diff * 0.5;
            reward += r_smoothness * 0.1;
            info.r_smoothness = r_smoothness;

            if distance < 0.5 {
                reward += 100.0;
                info.r_goal = 100.0;
                info.termination = Some("success".to_string());
            }

            if roll.abs() > 1.0 || pitch.abs() > 1.0 || pos.y < 0.1 {
                reward -= 100.0;
                info.termination = Some("fallen".to_string());
            }
        }

        (reward, info)
    }

    fn check_done(&self, info: &StepInfo) -> bool {
        info.termination.is_some() || self.step_count >= self.max_steps
    }
}

#[derive(Debug, Clone, Default)]
pub struct StepInfo {
    pub r_velocity: f32,
    pub r_distance: f32,
    pub r_stability: f32,
    pub r_energy: f32,
    pub r_smoothness: f32,
    pub r_goal: f32,
    pub termination: Option<String>,
}
