use crate::orbitalelements::*;
use nalgebra::*;
pub struct OtherBody {
    pub name: String,
    pub id: usize,
    pub mu: f64,
    pub pos_old: Vector3<f64>,
    pub mass: f64,
    pub state: Vector6<f64>,
    pub propagate_flag: bool,
    pub state_history: Vec<Vec<f64>>,
    pub coes: OrbitalElements,
}

#[allow(dead_code)]
impl OtherBody {
    pub fn new() -> Self {
        OtherBody {
            name: String::from(""),
            mass: 0.,
            mu: 0.,
            id: 1,
            pos_old: Vector3::zeros(),
            propagate_flag: true,
            state: vector![0., 0., 0., 0., 0., 0.,], // Assuming moon starts on the x-axis and other velocities will be set elsewhere
            state_history: vec![],
            coes: OrbitalElements {
                sma: 0.,
                ecc: 0.,
                inc: 0.,
                raan: 0.,
                aop: 0.,
                ta: 0.,
                param: 0.,
            },
        }
    }
}
