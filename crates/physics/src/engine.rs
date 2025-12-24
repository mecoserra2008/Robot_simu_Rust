use rapier3d::prelude::*;
use nalgebra as na;

pub type BodyHandle = RigidBodyHandle;
pub type ColliderHandle = rapier3d::geometry::ColliderHandle;

pub struct PhysicsEngine {
    pub integration_params: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub gravity: na::Vector3<f32>,
    pub dt: f32,
    pub time: f32,
}

impl PhysicsEngine {
    pub fn new(dt: f32, gravity: na::Vector3<f32>) -> Self {
        let mut integration_params = IntegrationParameters::default();
        integration_params.dt = dt;

        Self {
            integration_params,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            gravity,
            dt,
            time: 0.0,
        }
    }

    pub fn step(&mut self) {
        let gravity = self.gravity;
        let integration_params = self.integration_params;

        self.physics_pipeline.step(
            &gravity,
            &integration_params,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &(),
            &(),
        );

        self.query_pipeline.update(&self.rigid_body_set, &self.collider_set);
        self.time += self.dt;
    }

    pub fn create_rigid_body(&mut self, rb: RigidBody) -> BodyHandle {
        self.rigid_body_set.insert(rb)
    }

    pub fn create_collider(&mut self, collider: Collider, parent: BodyHandle) -> ColliderHandle {
        self.collider_set.insert_with_parent(collider, parent, &mut self.rigid_body_set)
    }

    pub fn get_body(&self, handle: BodyHandle) -> Option<&RigidBody> {
        self.rigid_body_set.get(handle)
    }

    pub fn get_body_mut(&mut self, handle: BodyHandle) -> Option<&mut RigidBody> {
        self.rigid_body_set.get_mut(handle)
    }

    pub fn apply_force(&mut self, handle: BodyHandle, force: na::Vector3<f32>) {
        if let Some(body) = self.rigid_body_set.get_mut(handle) {
            body.add_force(force, true);
        }
    }
}
