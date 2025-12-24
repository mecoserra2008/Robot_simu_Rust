use ndarray::Array1;

pub struct Transition {
    pub obs: Array1<f32>,
    pub action: Array1<f32>,
    pub reward: f32,
    pub next_obs: Array1<f32>,
    pub done: bool,
    pub log_prob: f32,
    pub value: f32,
}

pub struct RolloutBuffer {
    pub transitions: Vec<Transition>,
    pub capacity: usize,
}

impl RolloutBuffer {
    pub fn new(capacity: usize) -> Self {
        Self {
            transitions: Vec::with_capacity(capacity),
            capacity,
        }
    }

    pub fn push(&mut self, transition: Transition) {
        if self.transitions.len() < self.capacity {
            self.transitions.push(transition);
        } else {
            let idx = self.transitions.len() % self.capacity;
            self.transitions[idx] = transition;
        }
    }

    pub fn clear(&mut self) {
        self.transitions.clear();
    }

    pub fn len(&self) -> usize {
        self.transitions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.transitions.is_empty()
    }

    pub fn compute_gae(&self, gamma: f32, lambda: f32) -> (Vec<f32>, Vec<f32>) {
        let n = self.transitions.len();
        let mut advantages = vec![0.0; n];
        let mut returns = vec![0.0; n];

        let mut gae = 0.0;

        for t in (0..n).rev() {
            let trans = &self.transitions[t];

            let next_value = if t == n - 1 {
                0.0
            } else {
                self.transitions[t + 1].value
            };

            let delta = trans.reward + gamma * next_value * (1.0 - trans.done as i32 as f32) - trans.value;
            gae = delta + gamma * lambda * (1.0 - trans.done as i32 as f32) * gae;

            advantages[t] = gae;
            returns[t] = gae + trans.value;
        }

        (advantages, returns)
    }
}
