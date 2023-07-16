use crate::math::*;
use crate::pointmass::*;

pub struct CentralBody {
    mass: f64,
    equatorial_radius: f64,
    mu: f64,
    satellite: Body,
    step_width: f64,
    time: f64,
}

impl CentralBody {
    pub fn rk4(&self, state: &mut [f64;6]) {
        let mut state: [f64;6] = [0.;6];
        // let mut k1: [f64;6] = [0.;6];
        // let mut k2: [f64;6] = [0.;6];
        // let mut k3: [f64;6] = [0.;6];
        // let mut k4: [f64;6] = [0.;6];

        state[0..3].copy_from_slice(&self.satellite.position);
        state[3..6].copy_from_slice(&self.satellite.velocity);
        let k1 = self.satellite.dxdt(&state,&self.time);
        let k2_timearg = self.time + self.step_width / 2.;
        let mut k2_statearg: [f64;6] = [0.;6];
        for i in 0..k2_statearg.len() {
            k2_statearg[i] = state[i] + k1[i];
        }
        let k2 = self.satellite.dxdt(&k2_statearg,&k2_timearg);


    }
}
