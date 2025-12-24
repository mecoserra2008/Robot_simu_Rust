use crate::{ActorCritic, RolloutBuffer};

#[derive(Debug, Clone)]
pub struct PPOConfig {
    pub learning_rate: f32,
    pub gamma: f32,
    pub gae_lambda: f32,
    pub clip_epsilon: f32,
    pub entropy_coef: f32,
    pub value_coef: f32,
    pub max_grad_norm: f32,
    pub n_epochs: usize,
    pub batch_size: usize,
}

impl Default for PPOConfig {
    fn default() -> Self {
        Self {
            learning_rate: 0.0003,
            gamma: 0.99,
            gae_lambda: 0.95,
            clip_epsilon: 0.2,
            entropy_coef: 0.01,
            value_coef: 0.5,
            max_grad_norm: 0.5,
            n_epochs: 10,
            batch_size: 64,
        }
    }
}

pub struct PPOTrainer {
    pub network: ActorCritic,
    pub config: PPOConfig,
}

impl PPOTrainer {
    pub fn new(network: ActorCritic, config: PPOConfig) -> Self {
        Self { network, config }
    }

    pub fn train(&mut self, buffer: &RolloutBuffer) -> TrainMetrics {
        let (advantages, returns) = buffer.compute_gae(self.config.gamma, self.config.gae_lambda);

        let adv_mean = advantages.iter().sum::<f32>() / advantages.len() as f32;
        let adv_std = {
            let variance = advantages.iter()
                .map(|a| (a - adv_mean).powi(2))
                .sum::<f32>() / advantages.len() as f32;
            (variance + 1e-8).sqrt()
        };
        let normalized_advantages: Vec<f32> = advantages.iter()
            .map(|a| (a - adv_mean) / adv_std)
            .collect();

        let mut total_policy_loss = 0.0;
        let mut total_value_loss = 0.0;
        let mut total_entropy = 0.0;
        let mut n_updates = 0;

        for _ in 0..self.config.n_epochs {
            for i in 0..buffer.len() {
                let trans = &buffer.transitions[i];
                let advantage = normalized_advantages[i];
                let target_return = returns[i];

                let (_action_mean, value_pred) = self.network.forward(&trans.obs);

                let ratio: f32 = 1.0;
                let clipped_ratio = ratio.clamp(
                    1.0 - self.config.clip_epsilon,
                    1.0 + self.config.clip_epsilon,
                );

                let policy_loss_1 = -ratio * advantage;
                let policy_loss_2 = -clipped_ratio * advantage;
                let policy_loss = policy_loss_1.max(policy_loss_2);

                let value_loss = (value_pred - target_return).powi(2);

                let entropy = 0.01;

                let _loss = policy_loss + self.config.value_coef * value_loss - self.config.entropy_coef * entropy;

                let grad_scale = self.config.learning_rate;
                self.apply_gradients(grad_scale, policy_loss, value_loss);

                total_policy_loss += policy_loss;
                total_value_loss += value_loss;
                total_entropy += entropy;
                n_updates += 1;
            }
        }

        TrainMetrics {
            policy_loss: total_policy_loss / n_updates as f32,
            value_loss: total_value_loss / n_updates as f32,
            entropy: total_entropy / n_updates as f32,
        }
    }

    fn apply_gradients(&mut self, _grad_scale: f32, _policy_loss: f32, _value_loss: f32) {
        // Simplified gradient update (placeholder)
        // In a real implementation, you'd compute gradients and update network parameters
    }
}

#[derive(Debug, Clone)]
pub struct TrainMetrics {
    pub policy_loss: f32,
    pub value_loss: f32,
    pub entropy: f32,
}
