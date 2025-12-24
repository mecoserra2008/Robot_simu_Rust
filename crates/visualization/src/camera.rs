use bevy::prelude::*;
use bevy::input::mouse::{MouseMotion, MouseWheel};

#[derive(Component)]
pub struct CameraController {
    pub focus: Vec3,
    pub radius: f32,
    pub theta: f32,  // Horizontal angle
    pub phi: f32,    // Vertical angle
    pub sensitivity: f32,
    pub zoom_speed: f32,
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            focus: Vec3::new(0.0, 0.5, 0.0),
            radius: 5.0,
            theta: std::f32::consts::PI / 4.0,
            phi: std::f32::consts::PI / 6.0,
            sensitivity: 0.01,
            zoom_speed: 0.5,
        }
    }
}

pub fn setup_camera(mut commands: Commands) {
    let controller = CameraController::default();
    let position = controller.calculate_position();

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_translation(position)
                .looking_at(controller.focus, Vec3::Y),
            ..default()
        },
        controller,
    ));
}

impl CameraController {
    fn calculate_position(&self) -> Vec3 {
        let x = self.radius * self.phi.sin() * self.theta.cos();
        let y = self.radius * self.phi.cos();
        let z = self.radius * self.phi.sin() * self.theta.sin();

        self.focus + Vec3::new(x, y, z)
    }
}

pub fn camera_controller_system(
    mut mouse_motion: EventReader<MouseMotion>,
    mut mouse_wheel: EventReader<MouseWheel>,
    mouse_button: Res<ButtonInput<MouseButton>>,
    mut query: Query<(&mut Transform, &mut CameraController)>,
) {
    for (mut transform, mut controller) in query.iter_mut() {
        // Handle rotation
        if mouse_button.pressed(MouseButton::Right) {
            for motion in mouse_motion.read() {
                controller.theta -= motion.delta.x * controller.sensitivity;
                controller.phi -= motion.delta.y * controller.sensitivity;
                controller.phi = controller.phi.clamp(0.1, std::f32::consts::PI - 0.1);
            }
        }

        // Handle zoom
        for wheel in mouse_wheel.read() {
            controller.radius -= wheel.y * controller.zoom_speed;
            controller.radius = controller.radius.clamp(2.0, 50.0);
        }

        // Update camera position
        let new_position = controller.calculate_position();
        *transform = Transform::from_translation(new_position)
            .looking_at(controller.focus, Vec3::Y);
    }
}
