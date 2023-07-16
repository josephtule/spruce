// use crate::math::*;
use crate::pointmass::*;

pub struct DynamicalSystem<'a> {
    pub mass: f64,
    pub equatorial_radius: f64,
    pub mu: f64,
    pub satellite: &'a mut Body<'a>,
    pub step_width: f64,
    pub central_body: &'a CentralBody,    
    pub time: f64,
}

impl<'a> DynamicalSystem<'a> {
    pub fn rk4(&mut self) {
        let mut state: [f64;6] = [0.;6];
        // let mut k1: [f64;6] = [0.;6];
        // let mut k2: [f64;6] = [0.;6];
        // let mut k3: [f64;6] = [0.;6];
        // let mut k4: [f64;6] = [0.;6];

        state[0..3].copy_from_slice(&self.satellite.position);
        state[3..6].copy_from_slice(&self.satellite.velocity);
        // println!("The current state is {:.4?}",&state);
        let k1 = self.satellite.dxdt(&state,&self.time);
        let k2_timearg = self.time + self.step_width / 2.;
        let mut k2_statearg: [f64;6] = [0.;6];
        for i in 0..k2_statearg.len() {
            k2_statearg[i] = state[i] + self.step_width * k1[i]/2.;
        }
        let k2 = self.satellite.dxdt(&k2_statearg,&k2_timearg);
        let k3_timearg = self.time + self.step_width / 2.;
        let mut k3_statearg: [f64;6] = [0.;6];
        for i in 0..k3_statearg.len() {
            k3_statearg[i] = state[i] + self.step_width * k2[i]/2.;
        }
        let k3 = self.satellite.dxdt(&k3_statearg,&k3_timearg);
        let k4_timearg = self.time + self.step_width;
        let mut k4_statearg: [f64;6] = [0.;6];
        for i in 0..k4_statearg.len() {
            k4_statearg[i] = state[i] + self.step_width * k3[i];
        }
        let k4 = self.satellite.dxdt(&k4_statearg,&k4_timearg);
        for i in 0..3 {
            self.satellite.position[i] += self.step_width/6. * (k1[i] + 2.*k2[i] + 2.*k3[i] + k4[i]);
            self.satellite.velocity[i] += self.step_width/6. * (k1[i+3] + 2.*k2[i+3] + 2.*k3[i+3] + k4[i+3]);
        }
        self.time += self.step_width;
    }

}
