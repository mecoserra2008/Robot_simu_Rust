use ndarray::{Array1, Array2};
use rand::Rng;
use rand_distr::{Distribution, Normal};

pub struct MLP {
    pub layers: Vec<Layer>,
}

pub struct Layer {
    pub weights: Array2<f32>,
    pub biases: Array1<f32>,
    pub activation: Activation,
}

#[derive(Clone, Copy)]
pub enum Activation {
    ELU,
    Tanh,
    Identity,
}

impl MLP {
    pub fn new(layer_sizes: &[usize], activations: &[Activation]) -> Self {
        assert_eq!(layer_sizes.len() - 1, activations.len());

        let mut layers = Vec::new();
        let mut rng = rand::thread_rng();

        for i in 0..layer_sizes.len() - 1 {
            let in_size = layer_sizes[i];
            let out_size = layer_sizes[i + 1];

            let scale = (2.0 / in_size as f32).sqrt();
            let weights = Array2::from_shape_fn(
                (out_size, in_size),
                |_| rng.gen_range(-scale..scale),
            );
            let biases = Array1::zeros(out_size);

            layers.push(Layer {
                weights,
                biases,
                activation: activations[i],
            });
        }

        Self { layers }
    }

    pub fn forward(&self, input: &Array1<f32>) -> Array1<f32> {
        let mut x = input.clone();

        for layer in &self.layers {
            x = layer.weights.dot(&x) + &layer.biases;

            x.mapv_inplace(|v| match layer.activation {
                Activation::ELU => {
                    if v > 0.0 {
                        v
                    } else {
                        v.exp() - 1.0
                    }
                }
                Activation::Tanh => v.tanh(),
                Activation::Identity => v,
            });
        }

        x
    }

    pub fn param_count(&self) -> usize {
        self.layers.iter().map(|l| {
            l.weights.len() + l.biases.len()
        }).sum()
    }

    pub fn get_params(&self) -> Vec<f32> {
        let mut params = Vec::new();
        for layer in &self.layers {
            params.extend(layer.weights.iter());
            params.extend(layer.biases.iter());
        }
        params
    }

    pub fn set_params(&mut self, params: &[f32]) {
        let mut offset = 0;

        for layer in &mut self.layers {
            let w_len = layer.weights.len();
            let b_len = layer.biases.len();

            layer.weights.as_slice_mut().unwrap()
                .copy_from_slice(&params[offset..offset + w_len]);
            offset += w_len;

            layer.biases.as_slice_mut().unwrap()
                .copy_from_slice(&params[offset..offset + b_len]);
            offset += b_len;
        }
    }
}

pub struct ActorCritic {
    pub shared_encoder: MLP,
    pub actor_head: MLP,
    pub critic_head: MLP,
    pub log_std: Array1<f32>,
}

impl ActorCritic {
    pub fn new(obs_dim: usize, act_dim: usize, hidden_sizes: &[usize]) -> Self {
        use Activation::*;

        let mut encoder_sizes = vec![obs_dim];
        encoder_sizes.extend_from_slice(hidden_sizes);
        let encoder_acts = vec![ELU; hidden_sizes.len()];
        let shared_encoder = MLP::new(&encoder_sizes, &encoder_acts);

        let actor_head = MLP::new(
            &[*hidden_sizes.last().unwrap(), act_dim],
            &[Identity],
        );

        let critic_head = MLP::new(
            &[*hidden_sizes.last().unwrap(), 1],
            &[Identity],
        );

        let log_std = Array1::from_elem(act_dim, -0.5);

        Self {
            shared_encoder,
            actor_head,
            critic_head,
            log_std,
        }
    }

    pub fn forward(&self, obs: &Array1<f32>) -> (Array1<f32>, f32) {
        let features = self.shared_encoder.forward(obs);
        let action_mean = self.actor_head.forward(&features);
        let value = self.critic_head.forward(&features)[0];

        (action_mean, value)
    }

    pub fn sample_action(&self, obs: &Array1<f32>, rng: &mut impl Rng) -> (Array1<f32>, f32) {
        let (mean, _value) = self.forward(obs);
        let std = self.log_std.mapv(|x| x.exp());

        let mut action = Array1::zeros(mean.len());
        let mut log_prob = 0.0;

        for i in 0..mean.len() {
            let dist = Normal::new(mean[i], std[i]).unwrap();
            action[i] = dist.sample(rng);

            let normalized = (action[i] - mean[i]) / std[i];
            log_prob += -0.5 * normalized.powi(2) - std[i].ln() - 0.5 * (2.0 * std::f32::consts::PI).ln();
        }

        (action, log_prob)
    }
}
