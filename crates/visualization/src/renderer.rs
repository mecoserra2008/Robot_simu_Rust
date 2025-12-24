use bevy::prelude::*;
use bevy_rapier3d::prelude::*;

pub struct VisualizationPlugin;

impl Plugin for VisualizationPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Quadruped RL Simulation".to_string(),
                resolution: (1920., 1080.).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_systems(Startup, (setup_scene, setup_lighting))
        .add_systems(Update, (update_robot_visuals, update_objects, handle_input));
    }
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(shape::Plane::from_size(100.0).into()),
        material: materials.add(StandardMaterial {
            base_color: Color::rgb(0.3, 0.5, 0.3),
            perceptual_roughness: 0.9,
            ..default()
        }),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
    });

    // Grid helper
    for i in -10..=10 {
        let offset = i as f32 * 5.0;
        // X-axis lines
        commands.spawn(PbrBundle {
            mesh: meshes.add(shape::Box::new(100.0, 0.02, 0.02).into()),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(0.2, 0.2, 0.2),
                ..default()
            }),
            transform: Transform::from_xyz(0.0, 0.01, offset),
            ..default()
        });
        // Z-axis lines
        commands.spawn(PbrBundle {
            mesh: meshes.add(shape::Box::new(0.02, 0.02, 100.0).into()),
            material: materials.add(StandardMaterial {
                base_color: Color::rgb(0.2, 0.2, 0.2),
                ..default()
            }),
            transform: Transform::from_xyz(offset, 0.01, 0.0),
            ..default()
        });
    }
}

fn setup_lighting(mut commands: Commands) {
    // Directional light (sun)
    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // Ambient light
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 0.3,
    });

    // Point lights for better visibility
    for (x, z) in [(-10.0, -10.0), (10.0, -10.0), (-10.0, 10.0), (10.0, 10.0)] {
        commands.spawn(PointLightBundle {
            point_light: PointLight {
                intensity: 500.0,
                range: 50.0,
                shadows_enabled: false,
                ..default()
            },
            transform: Transform::from_xyz(x, 5.0, z),
            ..default()
        });
    }
}

fn update_robot_visuals(
    // TODO: Sync with physics state
) {
    // Update robot body and limb positions from physics
}

fn update_objects(
    // TODO: Sync object positions
) {
    // Update dynamic objects from physics
}

fn handle_input(
    keyboard: Res<Input<KeyCode>>,
    mut sim_state: ResMut<crate::SimulationState>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        sim_state.paused = !sim_state.paused;
    }
    if keyboard.just_pressed(KeyCode::Up) {
        sim_state.speed = (sim_state.speed * 1.5).min(10.0);
    }
    if keyboard.just_pressed(KeyCode::Down) {
        sim_state.speed = (sim_state.speed / 1.5).max(0.1);
    }
}

pub struct RobotVisuals;
pub struct ObjectVisuals;
