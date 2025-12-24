use bevy::prelude::*;
use bevy_rapier3d::prelude::*;
use physics::BodyHandle;

pub mod renderer;
pub mod ui;
pub mod camera;

pub use renderer::{VisualizationPlugin, RobotVisuals, ObjectVisuals};
pub use ui::{MetricsUI, TrainingStats};
pub use camera::{CameraController, setup_camera};

#[derive(Component)]
pub struct RobotBody {
    pub handle: BodyHandle,
}

#[derive(Component)]
pub struct RobotLeg {
    pub leg_index: usize,
}

#[derive(Component)]
pub struct RobotHead {
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Component)]
pub struct Gripper {
    pub is_open: bool,
    pub force: f32,
}

#[derive(Component)]
pub struct DynamicObject {
    pub handle: BodyHandle,
    pub health: f32,
    pub max_health: f32,
}

#[derive(Resource)]
pub struct SimulationState {
    pub paused: bool,
    pub speed: f32,
    pub episode: usize,
    pub total_reward: f32,
}
