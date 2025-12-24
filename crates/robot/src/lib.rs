pub mod actuator;
pub mod sensor;
pub mod quadruped;

pub use actuator::{Actuator, ActuatorSpec, ActuatorState, PDController};
pub use sensor::{IMU, Encoder, ContactSensor};
pub use quadruped::{QuadrupedRobot, Leg, Head, Gripper, RobotConfig, BodyConfig, LegsConfig, ActuatorConfig, ActuatorLimits};
