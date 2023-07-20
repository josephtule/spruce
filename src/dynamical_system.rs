// use crate::math::*;
use crate::pointmass::*;

pub struct DynamicalSystem<'a> {
    pub satellite: Vec<&'a mut Body<'a>>,
    pub step_width: f64,
    pub central_body: &'a CentralBody,
    pub time: f64,
}

impl<'a> DynamicalSystem<'a> {
    pub fn rk4(&mut self, sat_num: usize) {
        let halfstep = self.step_width / 2.;
        let k1 = self.satellite[sat_num].dxdt(&self.satellite[sat_num].state, &self.time);
        let k2_timearg = self.time + halfstep;
        let mut k2_statearg: [f64; 6] = [0.; 6];
        for i in 0..k2_statearg.len() {
            k2_statearg[i] = &self.satellite[sat_num].state[i] + k1[i] * halfstep;
        }
        let k2 = self.satellite[sat_num].dxdt(&k2_statearg, &k2_timearg);
        let k3_timearg = self.time + halfstep;
        let mut k3_statearg: [f64; 6] = [0.; 6];
        for i in 0..k3_statearg.len() {
            k3_statearg[i] = &self.satellite[sat_num].state[i] + k2[i] * halfstep;
        }
        let k3 = self.satellite[sat_num].dxdt(&k3_statearg, &k3_timearg);
        let k4_timearg = self.time + self.step_width;
        let mut k4_statearg: [f64; 6] = [0.; 6];
        for i in 0..k4_statearg.len() {
            k4_statearg[i] = &self.satellite[sat_num].state[i] + self.step_width * k3[i];
        }
        let k4 = self.satellite[sat_num].dxdt(&k4_statearg, &k4_timearg);
        for i in 0..6 {
            self.satellite[sat_num].state[i] +=
                self.step_width / 6. * (k1[i] + 2. * k2[i] + 2. * k3[i] + k4[i]);
        }
        self.time += self.step_width;
    }
    // asked chatgpt to speed this up, does it is not faster lol
    // pub fn rk4(&mut self, sat_num: usize) {
    //     let halfstep = self.step_width / 2.;
    //
    //     let k1 = self.satellite[sat_num].dxdt(&self.satellite[sat_num].state, &self.time);
    //
    //     let k2_statearg: [f64; 6] = self.satellite[sat_num]
    //         .state
    //         .iter()
    //         .zip(k1.iter())
    //         .map(|(state_val, k1_val)| state_val + k1_val * halfstep)
    //         .collect::<Vec<f64>>()
    //         .try_into()
    //         .unwrap();
    //     let k2 = self.satellite[sat_num].dxdt(&k2_statearg, &(self.time + halfstep));
    //
    //     let k3_statearg: [f64; 6] = self.satellite[sat_num]
    //         .state
    //         .iter()
    //         .zip(k2.iter())
    //         .map(|(state_val, k2_val)| state_val + k2_val * halfstep)
    //         .collect::<Vec<f64>>()
    //         .try_into()
    //         .unwrap();
    //     let k3 = self.satellite[sat_num].dxdt(&k3_statearg, &(self.time + halfstep));
    //
    //     let k4_statearg: [f64; 6] = self.satellite[sat_num]
    //         .state
    //         .iter()
    //         .zip(k3.iter())
    //         .map(|(state_val, k3_val)| state_val + k3_val * self.step_width)
    //         .collect::<Vec<f64>>()
    //         .try_into()
    //         .unwrap();
    //     let k4 = self.satellite[sat_num].dxdt(&k4_statearg, &(self.time + self.step_width));
    //
    //     for i in 0..6 {
    //         self.satellite[sat_num].state[i] +=
    //             self.step_width / 6. * (k1[i] + 2. * k2[i] + 2. * k3[i] + k4[i]);
    //     }
    //     self.time += self.step_width;
    // }
}
