use physics::na;
use std::collections::BinaryHeap;
use std::cmp::Ordering;

#[derive(Debug, Clone, PartialEq)]
pub enum GoalType {
    Navigate { target: na::Vector3<f32>, urgency: f32 },
    AvoidThreat { threat_pos: na::Vector3<f32>, threat_velocity: na::Vector3<f32>, urgency: f32 },
    GrabObject { object_id: usize, priority: f32 },
    PlaceObject { target_pos: na::Vector3<f32>, priority: f32 },
    Survive { health_threshold: f32, urgency: f32 },
    Explore { region: na::Vector3<f32>, curiosity: f32 },
    RestoreHealth { urgency: f32 },
}

impl GoalType {
    pub fn get_priority(&self) -> f32 {
        match self {
            GoalType::Survive { urgency, .. } => 100.0 * urgency,
            GoalType::AvoidThreat { urgency, .. } => 80.0 * urgency,
            GoalType::RestoreHealth { urgency } => 70.0 * urgency,
            GoalType::Navigate { urgency, .. } => 50.0 * urgency,
            GoalType::GrabObject { priority, .. } => 40.0 * priority,
            GoalType::PlaceObject { priority, .. } => 35.0 * priority,
            GoalType::Explore { curiosity, .. } => 20.0 * curiosity,
        }
    }

    pub fn get_expected_reward(&self) -> f32 {
        match self {
            GoalType::Survive { .. } => 1000.0,
            GoalType::AvoidThreat { .. } => 500.0,
            GoalType::RestoreHealth { .. } => 300.0,
            GoalType::Navigate { .. } => 100.0,
            GoalType::GrabObject { .. } => 150.0,
            GoalType::PlaceObject { .. } => 200.0,
            GoalType::Explore { .. } => 50.0,
        }
    }

    pub fn get_time_constraint(&self) -> Option<f32> {
        match self {
            GoalType::Survive { .. } => Some(1.0),  // 1 second
            GoalType::AvoidThreat { .. } => Some(2.0),  // 2 seconds
            GoalType::RestoreHealth { .. } => Some(5.0),  // 5 seconds
            _ => None,  // No time constraint
        }
    }
}

#[derive(Debug, Clone)]
pub struct Goal {
    pub goal_type: GoalType,
    pub creation_time: f32,
    pub deadline: Option<f32>,
    pub active: bool,
}

impl Goal {
    pub fn new(goal_type: GoalType, current_time: f32) -> Self {
        let deadline = goal_type.get_time_constraint().map(|t| current_time + t);
        Self {
            goal_type,
            creation_time: current_time,
            deadline,
            active: true,
        }
    }

    pub fn is_expired(&self, current_time: f32) -> bool {
        self.deadline.map_or(false, |d| current_time > d)
    }

    pub fn get_effective_priority(&self, current_time: f32) -> f32 {
        let base_priority = self.goal_type.get_priority();

        // Increase priority as deadline approaches
        if let Some(deadline) = self.deadline {
            let time_left = (deadline - current_time).max(0.0);
            let time_since_creation = current_time - self.creation_time;
            let total_time = time_since_creation + time_left;

            if total_time > 0.0 {
                let urgency_mult = 1.0 + (1.0 - time_left / total_time);
                base_priority * urgency_mult
            } else {
                base_priority * 2.0
            }
        } else {
            base_priority
        }
    }
}

impl PartialEq for Goal {
    fn eq(&self, other: &Self) -> bool {
        self.creation_time == other.creation_time
    }
}

impl Eq for Goal {}

impl PartialOrd for Goal {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Goal {
    fn cmp(&self, other: &Self) -> Ordering {
        // Higher priority goals come first
        other.goal_type.get_priority()
            .partial_cmp(&self.goal_type.get_priority())
            .unwrap_or(Ordering::Equal)
    }
}

pub struct GoalManager {
    goals: BinaryHeap<Goal>,
    current_time: f32,
}

impl GoalManager {
    pub fn new() -> Self {
        Self {
            goals: BinaryHeap::new(),
            current_time: 0.0,
        }
    }

    pub fn add_goal(&mut self, goal_type: GoalType) {
        let goal = Goal::new(goal_type, self.current_time);
        self.goals.push(goal);
    }

    pub fn update(&mut self, dt: f32) {
        self.current_time += dt;

        // Remove expired goals
        self.goals.retain(|goal| goal.active && !goal.is_expired(self.current_time));
    }

    pub fn get_current_goal(&self) -> Option<&Goal> {
        self.goals.peek()
    }

    pub fn get_top_goals(&self, n: usize) -> Vec<&Goal> {
        self.goals.iter().take(n).collect()
    }

    pub fn complete_goal(&mut self, goal_type: &GoalType) {
        self.goals.retain(|g| &g.goal_type != goal_type);
    }

    pub fn get_goal_count(&self) -> usize {
        self.goals.len()
    }

    pub fn clear_goals(&mut self) {
        self.goals.clear();
    }
}

impl Default for GoalManager {
    fn default() -> Self {
        Self::new()
    }
}
