use environment::gym_env::{RobotEnv, OBS_DIM, ACT_DIM};
use rl::{ActorCritic, PPOTrainer, PPOConfig, RolloutBuffer};
use robot::RobotConfig;
use std::fs;
use tracing::info;
use ndarray::Array1;

fn main() -> anyhow::Result<()> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_env_filter("info")
        .init();

    info!("🤖 Quadruped RL Training Started");

    // Load robot config
    let robot_config_str = fs::read_to_string("config/robot_spec.toml")?;
    let robot_config: toml::Value = toml::from_str(&robot_config_str)?;

    // Extract the necessary fields for RobotConfig
    let robot_config = RobotConfig {
        body: robot::BodyConfig {
            mass: robot_config["body"]["mass"].as_float().unwrap() as f32,
            dimensions: robot_config["body"]["dimensions"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_float().unwrap() as f32)
                .collect(),
            surface_friction: robot_config["body"]["surface_friction"].as_float().unwrap() as f32,
            restitution: robot_config["body"]["restitution"].as_float().unwrap() as f32,
        },
        legs: robot::LegsConfig {
            positions: robot_config["legs"]["positions"]
                .as_array()
                .unwrap()
                .iter()
                .map(|arr| {
                    arr.as_array()
                        .unwrap()
                        .iter()
                        .map(|v| v.as_float().unwrap() as f32)
                        .collect()
                })
                .collect(),
        },
        actuators: robot::ActuatorConfig {
            continuous_torque: robot_config["actuators"]["continuous_torque"].as_float().unwrap() as f32,
            peak_torque: robot_config["actuators"]["peak_torque"].as_float().unwrap() as f32,
            max_velocity: robot_config["actuators"]["max_velocity"].as_float().unwrap() as f32,
            rotor_inertia: robot_config["actuators"]["rotor_inertia"].as_float().unwrap() as f32,
            gear_ratio: robot_config["actuators"]["gear_ratio"].as_float().unwrap() as f32,
            motor_constant: robot_config["actuators"]["motor_constant"].as_float().unwrap() as f32,
            resistance: robot_config["actuators"]["resistance"].as_float().unwrap() as f32,
            damping: robot_config["actuators"]["damping"].as_float().unwrap() as f32,
            friction: robot_config["actuators"]["friction"].as_float().unwrap() as f32,
            backlash: robot_config["actuators"]["backlash"].as_float().unwrap() as f32,
            limits: robot::ActuatorLimits {
                hip_x: robot_config["actuators"]["limits"]["hip_x"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|v| v.as_float().unwrap() as f32)
                    .collect(),
                hip_y: robot_config["actuators"]["limits"]["hip_y"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|v| v.as_float().unwrap() as f32)
                    .collect(),
                knee: robot_config["actuators"]["limits"]["knee"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|v| v.as_float().unwrap() as f32)
                    .collect(),
            },
        },
    };

    // Load objects
    let objects_config_str = fs::read_to_string("config/objects.toml")?;
    let objects_value: toml::Value = toml::from_str(&objects_config_str)?;
    let objects_array = objects_value["objects"].as_array().unwrap();

    let mut objects_config = Vec::new();
    for obj in objects_array {
        let obj_config = environment::ObjectConfig {
            id: obj["id"].as_str().unwrap().to_string(),
            mass: obj.get("mass").and_then(|v| v.as_float()).map(|f| f as f32),
            dimensions: obj.get("dimensions").and_then(|v| v.as_array()).map(|arr| {
                arr.iter().map(|v| v.as_float().unwrap() as f32).collect()
            }),
            height: obj.get("height").and_then(|v| v.as_float()).map(|f| f as f32),
            width: obj.get("width").and_then(|v| v.as_float()).map(|f| f as f32),
            shape: obj["shape"].as_str().unwrap().to_string(),
            friction: obj["friction"].as_float().unwrap() as f32,
            restitution: obj["restitution"].as_float().unwrap() as f32,
            static_obj: obj.get("static_obj").and_then(|v| v.as_bool()).unwrap_or(false),
            movement_pattern: obj.get("movement_pattern").and_then(|v| v.as_str()).map(|s| s.to_string()),
            speed_range: obj.get("speed_range").and_then(|v| v.as_array()).map(|arr| {
                arr.iter().map(|v| v.as_float().unwrap() as f32).collect()
            }),
        };
        objects_config.push(obj_config);
    }

    // Create environment
    info!("Creating environment...");
    let mut env = RobotEnv::new(robot_config, objects_config);

    // Create network
    info!("Creating neural network (obs_dim={}, act_dim={})...", OBS_DIM, ACT_DIM);
    let network = ActorCritic::new(OBS_DIM, ACT_DIM, &[256, 128]);

    // Create trainer
    let mut trainer = PPOTrainer::new(network, PPOConfig::default());

    // Training parameters
    let n_episodes = 1000;
    let rollout_steps = 256;

    info!("Starting training loop ({} episodes)...", n_episodes);

    for episode in 0..n_episodes {
        let mut rollout_buffer = RolloutBuffer::new(rollout_steps);
        let mut obs = env.reset();
        let mut episode_reward = 0.0;
        let mut episode_steps = 0;
        let mut rng = rand::thread_rng();

        // Collect rollout
        for _step in 0..rollout_steps {
            let obs_array = Array1::from_vec(obs.clone());

            // Sample action from policy
            let (action, log_prob) = trainer.network.sample_action(&obs_array, &mut rng);
            let (_mean, value) = trainer.network.forward(&obs_array);

            // Step environment
            let (next_obs, reward, done, _info) = env.step(action.as_slice().unwrap());

            episode_reward += reward;
            episode_steps += 1;

            // Store transition
            rollout_buffer.push(rl::Transition {
                obs: obs_array,
                action: action.clone(),
                reward,
                next_obs: Array1::from_vec(next_obs.clone()),
                done,
                log_prob,
                value,
            });

            obs = next_obs;

            if done {
                obs = env.reset();
            }
        }

        // Train policy
        let metrics = trainer.train(&rollout_buffer);

        // Logging
        if episode % 10 == 0 {
            info!(
                "Episode {:4}: AvgReward={:7.2}, Steps={:4}, PolicyLoss={:.4}, ValueLoss={:.4}",
                episode,
                episode_reward / episode_steps as f32,
                episode_steps,
                metrics.policy_loss,
                metrics.value_loss,
            );
        }

        // Save checkpoint
        if episode % 100 == 0 && episode > 0 {
            info!("💾 Checkpoint at episode {}", episode);
        }
    }

    info!("✅ Training complete!");

    Ok(())
}
