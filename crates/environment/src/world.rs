use physics::{PhysicsEngine, BodyHandle, na, RigidBodyBuilder, ColliderBuilder, vector};
use serde::Deserialize;
use rand::Rng;

#[derive(Debug, Clone, Deserialize)]
pub struct ObjectConfig {
    pub id: String,
    #[serde(default)]
    pub mass: Option<f32>,
    #[serde(default)]
    pub dimensions: Option<Vec<f32>>,
    #[serde(default)]
    pub height: Option<f32>,
    #[serde(default)]
    pub width: Option<f32>,
    pub shape: String,
    pub friction: f32,
    pub restitution: f32,
    #[serde(default)]
    pub static_obj: bool,
    #[serde(default)]
    pub movement_pattern: Option<String>,
    #[serde(default)]
    pub speed_range: Option<Vec<f32>>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum MassValue {
    Finite(f32),
    Infinite(String),
}

pub struct DynamicObject {
    pub config: ObjectConfig,
    pub body_handle: BodyHandle,
    pub velocity_target: na::Vector3<f32>,
}

pub struct World {
    pub ground_handle: BodyHandle,
    pub objects: Vec<DynamicObject>,
    pub target_position: na::Vector3<f32>,
}

impl World {
    pub fn new(physics: &mut PhysicsEngine, object_configs: Vec<ObjectConfig>) -> Self {
        let ground = RigidBodyBuilder::fixed()
            .translation(vector![0.0, 0.0, 0.0])
            .build();
        let ground_handle = physics.create_rigid_body(ground);

        let ground_collider = ColliderBuilder::cuboid(50.0, 0.1, 50.0)
            .friction(0.8)
            .restitution(0.1)
            .build();
        physics.create_collider(ground_collider, ground_handle);

        let mut objects = Vec::new();
        for config in object_configs {
            if let Some(obj) = Self::create_object(physics, &config) {
                objects.push(obj);
            }
        }

        Self {
            ground_handle,
            objects,
            target_position: na::Vector3::new(5.0, 0.0, 0.0),
        }
    }

    fn create_object(physics: &mut PhysicsEngine, config: &ObjectConfig) -> Option<DynamicObject> {
        let is_static = config.static_obj;

        let rigid_body = if is_static {
            RigidBodyBuilder::fixed()
        } else {
            RigidBodyBuilder::dynamic()
        }
        .translation(vector![
            rand::thread_rng().gen_range(-10.0..10.0),
            0.5,
            rand::thread_rng().gen_range(-10.0..10.0)
        ])
        .build();

        let body_handle = physics.create_rigid_body(rigid_body);

        let collider = match config.shape.as_str() {
            "box" => {
                let dims = config.dimensions.as_ref()?;
                ColliderBuilder::cuboid(dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0)
                    .friction(config.friction)
                    .restitution(config.restitution)
                    .build()
            }
            "capsule" => {
                let height = config.height?;
                let width = config.width?;
                ColliderBuilder::capsule_y(height / 2.0, width / 2.0)
                    .friction(config.friction)
                    .restitution(config.restitution)
                    .build()
            }
            "cylinder" => {
                let height = config.height?;
                ColliderBuilder::cylinder(height / 2.0, 0.3)
                    .friction(config.friction)
                    .restitution(config.restitution)
                    .build()
            }
            _ => return None,
        };

        physics.create_collider(collider, body_handle);

        Some(DynamicObject {
            config: config.clone(),
            body_handle,
            velocity_target: na::Vector3::zeros(),
        })
    }

    pub fn update(&mut self, physics: &mut PhysicsEngine, _dt: f32) {
        for obj in &mut self.objects {
            if let Some(ref pattern) = obj.config.movement_pattern {
                match pattern.as_str() {
                    "bipedal_walk" | "quadruped_trot" => {
                        if rand::thread_rng().gen::<f32>() < 0.01 {
                            let speed_range = obj.config.speed_range.as_ref().unwrap();
                            let speed = rand::thread_rng().gen_range(speed_range[0]..speed_range[1]);
                            let angle = rand::thread_rng().gen_range(0.0..std::f32::consts::TAU);

                            obj.velocity_target = na::Vector3::new(
                                speed * angle.cos(),
                                0.0,
                                speed * angle.sin(),
                            );
                        }

                        if let Some(body) = physics.get_body_mut(obj.body_handle) {
                            body.set_linvel(obj.velocity_target, true);
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}
