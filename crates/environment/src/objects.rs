use physics::{PhysicsEngine, BodyHandle, na, RigidBodyBuilder, ColliderBuilder, vector};
use serde::Deserialize;
use rand::Rng;

#[derive(Debug, Clone, Deserialize)]
pub enum ObjectShape {
    Box { dimensions: [f32; 3] },
    Sphere { radius: f32 },
    Capsule { height: f32, radius: f32 },
    Cylinder { height: f32, radius: f32 },
    Cone { height: f32, radius: f32 },
}

#[derive(Debug, Clone, Deserialize)]
pub enum MaterialType {
    Wood,
    Metal,
    Glass,
    Rubber,
    Plastic,
    Stone,
}

impl MaterialType {
    pub fn get_properties(&self) -> (f32, f32, f32, f32) {
        // Returns: (density, friction, restitution, health_multiplier)
        match self {
            MaterialType::Wood => (600.0, 0.6, 0.3, 0.8),
            MaterialType::Metal => (7800.0, 0.7, 0.5, 2.0),
            MaterialType::Glass => (2500.0, 0.4, 0.1, 0.3),
            MaterialType::Rubber => (1100.0, 1.2, 0.9, 1.0),
            MaterialType::Plastic => (950.0, 0.5, 0.4, 0.7),
            MaterialType::Stone => (2600.0, 0.8, 0.2, 1.5),
        }
    }
}

#[derive(Debug, Clone)]
pub struct PhysicsObject {
    pub id: String,
    pub shape: ObjectShape,
    pub material: MaterialType,
    pub body_handle: BodyHandle,
    pub mass: f32,
    pub health: f32,
    pub max_health: f32,
    pub is_dangerous: bool,
    pub damage_on_contact: f32,
    pub velocity_history: Vec<na::Vector3<f32>>,
}

impl PhysicsObject {
    pub fn new(
        physics: &mut PhysicsEngine,
        id: String,
        shape: ObjectShape,
        material: MaterialType,
        is_static: bool,
    ) -> Self {
        let (density, friction, restitution, health_mult) = material.get_properties();

        let (body_handle, mass, volume) = Self::create_body(physics, &shape, density, friction, restitution, is_static);

        let health = volume * health_mult * 100.0;

        Self {
            id,
            shape,
            material,
            body_handle,
            mass,
            health,
            max_health: health,
            is_dangerous: false,
            damage_on_contact: 0.0,
            velocity_history: Vec::new(),
        }
    }

    fn create_body(
        physics: &mut PhysicsEngine,
        shape: &ObjectShape,
        density: f32,
        friction: f32,
        restitution: f32,
        is_static: bool,
    ) -> (BodyHandle, f32, f32) {
        let mut rng = rand::thread_rng();

        let rigid_body = if is_static {
            RigidBodyBuilder::fixed()
        } else {
            RigidBodyBuilder::dynamic()
        }
        .translation(vector![
            rng.gen_range(-5.0..5.0),
            rng.gen_range(1.0..3.0),
            rng.gen_range(-5.0..5.0)
        ])
        .build();

        let body_handle = physics.create_rigid_body(rigid_body);

        let (collider, volume) = match shape {
            ObjectShape::Box { dimensions } => {
                let vol = dimensions[0] * dimensions[1] * dimensions[2];
                (
                    ColliderBuilder::cuboid(dimensions[0] / 2.0, dimensions[1] / 2.0, dimensions[2] / 2.0)
                        .density(density)
                        .friction(friction)
                        .restitution(restitution)
                        .build(),
                    vol,
                )
            }
            ObjectShape::Sphere { radius } => {
                let vol = 4.0 / 3.0 * std::f32::consts::PI * radius.powi(3);
                (
                    ColliderBuilder::ball(*radius)
                        .density(density)
                        .friction(friction)
                        .restitution(restitution)
                        .build(),
                    vol,
                )
            }
            ObjectShape::Capsule { height, radius } => {
                let vol = std::f32::consts::PI * radius.powi(2) * (height + 4.0 / 3.0 * radius);
                (
                    ColliderBuilder::capsule_y(height / 2.0, *radius)
                        .density(density)
                        .friction(friction)
                        .restitution(restitution)
                        .build(),
                    vol,
                )
            }
            ObjectShape::Cylinder { height, radius } => {
                let vol = std::f32::consts::PI * radius.powi(2) * height;
                (
                    ColliderBuilder::cylinder(height / 2.0, *radius)
                        .density(density)
                        .friction(friction)
                        .restitution(restitution)
                        .build(),
                    vol,
                )
            }
            ObjectShape::Cone { height, radius } => {
                let vol = 1.0 / 3.0 * std::f32::consts::PI * radius.powi(2) * height;
                (
                    ColliderBuilder::cone(height / 2.0, *radius)
                        .density(density)
                        .friction(friction)
                        .restitution(restitution)
                        .build(),
                    vol,
                )
            }
        };

        physics.create_collider(collider, body_handle);

        let mass = volume * density;
        (body_handle, mass, volume)
    }

    pub fn update(&mut self, physics: &PhysicsEngine, _dt: f32) {
        if let Some(body) = physics.get_body(self.body_handle) {
            let velocity = na::Vector3::new(body.linvel().x, body.linvel().y, body.linvel().z);
            self.velocity_history.push(velocity);

            // Keep only last 10 velocities
            if self.velocity_history.len() > 10 {
                self.velocity_history.remove(0);
            }

            // Check if object is falling dangerously
            let speed = velocity.norm();
            if speed > 5.0 && velocity.y < -2.0 {
                self.is_dangerous = true;
                self.damage_on_contact = (speed * self.mass / 100.0).min(50.0);
            } else {
                self.is_dangerous = false;
                self.damage_on_contact = 0.0;
            }
        }
    }

    pub fn take_damage(&mut self, amount: f32) -> bool {
        self.health -= amount;
        self.health <= 0.0
    }

    pub fn is_fragile(&self) -> bool {
        matches!(self.material, MaterialType::Glass)
    }
}
