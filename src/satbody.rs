// use crate::math::*;
use nalgebra::*;
// use std::ops::AddAssign;
// use std::time::Instant;

pub struct SatBody {
    pub name: String,
    pub mass: f64,
    pub state: Vector6<f64>,
    pub propagate_flag: bool,
    pub state_history: Vec<Vec<f64>>,
    pub time_history: Vec<f64>,
}

#[allow(dead_code)]
impl SatBody {
    pub fn new() -> Self {
        SatBody {
            name: String::from(""),                  // match struct name
            mass: 0.,                                //kg
            state: vector![0., 0., 0., 0., 0., 0.,], // m, m/s
            propagate_flag: true,
            state_history: vec![],
            time_history: vec![0.],
        }
    }
}
