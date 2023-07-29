// use crate::math::*;
use crate::centralbody::*;
use crate::otherbody::*;
use crate::satbody::*;
use nalgebra::*;
use std::fs::File;
use std::io::Write;
use std::time::Instant;
// use nalgebra::*;
pub struct DynamicalSystem<'a> {
    pub satellite: Vec<&'a mut SatBody<'a>>,
    pub central_body: &'a CentralBody,
    pub other_body: Vec<&'a mut OtherBody>,
    pub time: f64,
    pub step_width: f64,
    pub maxsteps: usize,
    pub writeflag: bool,
    pub timeflag: bool,
    pub storeflag: bool,
}

impl<'a> DynamicalSystem<'a> {
    pub fn propagate(&mut self) {
        let mut files: Vec<File> = Vec::new();

        let start_time = Instant::now();

        for sat_num in 0..self.satellite.len() {
            // propagate index
            let mut i = 1;

            // add file to files vector
            let filename = format!("output{}.txt", sat_num);
            let file = File::create(&filename).expect("Failed to create file");
            files.push(file);

            let init_pos = self.satellite[sat_num].state.data.0[0];

            // store initial position
            self.satellite[sat_num]
                .state_history
                .push(init_pos.to_vec());

            //write first line
            if self.writeflag {
                writeln!(
                    files[sat_num],
                    "{}, {}, {}",
                    &self.satellite[sat_num].state[0],
                    &self.satellite[sat_num].state[1],
                    &self.satellite[sat_num].state[2]
                )
                .expect("Failed to write to file");
                // println!("The new position is {:.4?}",&self.satellite.position);
            }
            // propagate for current satellite and write to file
            loop {
                self.rk4(sat_num);
                if self.writeflag {
                    writeln!(
                        files[sat_num],
                        "{}, {}, {}",
                        &self.satellite[sat_num].state[0],
                        &self.satellite[sat_num].state[1],
                        &self.satellite[sat_num].state[2]
                    )
                    .expect("Failed to write to file");
                    // println!("The new position is {:.4?}",&self.satellite.position);
                }

                i += 1;
                if i > self.maxsteps {
                    break;
                }
            }
        }
        if self.timeflag == true {
            let end_time = Instant::now() - start_time;
            println!("Elapsed time: {:?}", end_time);
        }
    }

    pub fn rk4(&mut self, sat_num: usize) {
        let halfstep = self.step_width / 2.;

        let k1 = self.satellite[sat_num]
            .gravity
            .dxdt(&self.satellite[sat_num].state, &self.time);

        let k2 = self.satellite[sat_num].gravity.dxdt(
            &(self.satellite[sat_num].state + k1 * halfstep),
            &(self.time + halfstep),
        );

        let k3 = self.satellite[sat_num].gravity.dxdt(
            &(self.satellite[sat_num].state + k2 * halfstep),
            &(self.time + halfstep),
        );

        let k4 = self.satellite[sat_num].gravity.dxdt(
            &(self.satellite[sat_num].state + self.step_width * k3),
            &(self.time + self.step_width),
        );

        let state_new =
            self.satellite[sat_num].state + self.step_width / 6. * (k1 + 2. * k2 + 2. * k3 + k4);
        self.satellite[sat_num].state = state_new;
        let time_new = self.time + self.step_width;
        self.time = time_new;

        // store state and time histories
        if self.storeflag {
            self.satellite[sat_num]
                .state_history
                .push(state_new.data.0[0].to_vec());
            self.satellite[sat_num].time_history.push(time_new);
        }
    }
    #[allow(dead_code)]
    pub fn rk4_step(
        &self,
        dxdt: &dyn Fn(&Vector6<f64>, &f64) -> Vector6<f64>,
        state: &Vector6<f64>,
    ) -> Vector6<f64> {
        let halfstep = self.step_width / 2.;

        let k1 = dxdt(&state, &self.time);

        let k2 = dxdt(&(state + k1 * halfstep), &(self.time + halfstep));

        let k3 = dxdt(&(state + k2 * halfstep), &(self.time + halfstep));

        let k4 = dxdt(
            &(state + self.step_width * k3),
            &(self.time + self.step_width),
        );

        state + self.step_width / 6. * (k1 + 2. * k2 + 2. * k3 + k4)
    }

    pub fn propagate3(&mut self) {
        let mut files: Vec<File> = Vec::new();

        let start_time = Instant::now();

        // storing and/or writing initial states for satellites
        for sat_num in 0..self.satellite.len() {
            if self.storeflag {
                // store first state
                let init_state = self.satellite[sat_num].state.data.0[0];
                self.satellite[sat_num]
                    .state_history
                    .push(init_state.to_vec());
            }
            if self.writeflag {
                // init file
                let filename = format!("output{}.txt", sat_num);
                let file = File::create(&filename).expect("Failed to create file");
                files.push(file);

                // write first position
                writeln!(
                    files[sat_num],
                    "{}, {}, {}",
                    &self.satellite[sat_num].state[0],
                    &self.satellite[sat_num].state[1],
                    &self.satellite[sat_num].state[2]
                )
                .expect("Failed to write to file");
            }
        }
        // storing initial states for other bodies
        for other_num in 0..self.other_body.len() {
            if self.storeflag {
                // store first state
                let init_state = self.gravity.other_grav.other_body[other_num].state.data.0[0];
                self.gravity.other_grav.other_body[other_num]
                    .state_history
                    .push(init_state.to_vec());
            }

            let mut init_pos = Vector3::zeros();
            init_pos.fixed_rows_mut::<3usize>(0).copy_from(
                &self.gravity.other_grav.other_body[other_num]
                    .state
                    .fixed_rows::<3usize>(0),
            );
            self.gravity.other_grav.other_body[other_num]
                .pos_old
                .fixed_rows_mut::<3usize>(0)
                .copy_from(&init_pos);
        }

        // propagation ----------------------------------------------------------------------
        for k in 0..self.maxsteps {
            // integrate for each satellite
            for sat_num in 0..self.satellite.len() {
                self.rk4_sat(sat_num);

                let current_state = self.satellite[sat_num].state.clone();

                let dxdt_fun = |state: &Vector6<f64>, time: &f64| -> Vector6<f64> {
                    self.gravity.dxdt(state, time)
                };

                let state_new = self.rk4_step(&dxdt_fun, &current_state);

                self.satellite[sat_num].state = state_new;
                // store state and time histories
                if self.storeflag {
                    let state_new = self.satellite[sat_num].state;
                    self.satellite[sat_num]
                        .state_history
                        .push(state_new.data.0[0].to_vec());
                    let time_new = self.time + self.step_width;
                    self.satellite[sat_num].time_history.push(time_new);
                }

                if self.writeflag {
                    writeln!(
                        files[sat_num],
                        "{}, {}, {}",
                        &self.satellite[sat_num].state[0],
                        &self.satellite[sat_num].state[1],
                        &self.satellite[sat_num].state[2]
                    )
                    .expect("Failed to write to file");
                    // println!("The new position is {:.4?}",&self.satellite.position);
                }
            }
            for other_num in 0..self.gravity.other_grav.other_body.len() {
                // store old position before integration step
                let mut old_pos = Vector3::zeros();
                old_pos.fixed_rows_mut::<3usize>(0).copy_from(
                    &self.gravity.other_grav.other_body[other_num]
                        .state
                        .fixed_rows::<3usize>(0),
                );
                self.gravity.other_grav.other_body[other_num]
                    .pos_old
                    .fixed_rows_mut::<3usize>(0)
                    .copy_from(&old_pos);

                let current_state = self.gravity.other_grav.other_body[other_num].state.clone();
                let dxdt_fun = |state: &Vector6<f64>, time: &f64| -> Vector6<f64> {
                    self.gravity.other_grav.dxdt(state, time, other_num)
                };

                let state_new = self.rk4_step(&dxdt_fun, &current_state);

                self.gravity.other_grav.other_body[other_num].state = state_new;

                // store state and time histories
                if self.storeflag {
                    let state_new = self.gravity.other_grav.other_body[other_num].state;
                    self.gravity.other_grav.other_body[other_num]
                        .state_history
                        .push(state_new.data.0[0].to_vec());
                    let time_new = self.time + self.step_width;
                    self.gravity.other_grav.other_body[other_num]
                        .time_history
                        .push(time_new);
                }
            }
        }

        if self.timeflag == true {
            let end_time = Instant::now() - start_time;
            println!("Elapsed time: {:?}", end_time);
        }
    }
}
