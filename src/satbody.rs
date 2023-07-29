// use crate::math::*;
use crate::centralbody::*;
use nalgebra::*;
// use std::ops::AddAssign;
// use std::time::Instant;

pub struct SatBody<'a> {
    pub name: String,
    pub mass: f64,
    pub state: Vector6<f64>,
    pub propagate_flag: bool,
    pub central_body: &'a CentralBody,
    pub state_history: Vec<Vec<f64>>,
    pub time_history: Vec<f64>,
}

#[allow(dead_code)]
impl<'a> SatBody<'a> {}
