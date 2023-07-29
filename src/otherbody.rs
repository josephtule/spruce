use crate::centralbody::*;
use nalgebra::*;

pub struct OtherBody<'a> {
    pub id: usize,
    pub central_body: &'a CentralBody,
    pub mu: f64,
    pub pos_old: Vector3<f64>,
    pub mass: f64,
    pub state: Vector6<f64>,
    pub propagate_flag: bool,
    pub state_history: Vec<Vec<f64>>,
    pub time_history: Vec<f64>,
}
