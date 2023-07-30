// use crate::math::*;
use crate::centralbody::*;
use crate::eoms::*;
// use crate::otherbody::*;
// use crate::satbody::*;
use nalgebra::*;
use std::fs::File;
use std::io::Write;
use std::time::Instant;
// use nalgebra::*;
pub struct DynamicalSystem<'a> {
    pub central_body: &'a CentralBody,
    pub eoms: &'a mut Eoms<'a>,
    pub time: f64,
    pub step_width: f64,
    pub maxsteps: usize,
    pub writeflag: bool,
    pub timeflag: bool,
    pub storeflag: bool,
}

impl<'a> DynamicalSystem<'a> {
    #[allow(dead_code)]
    pub fn rk4_step_bak(
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

    pub fn rk4_step<F>(&self, dxdt: F, state: &Vector6<f64>) -> Vector6<f64>
    where
        F: Fn(&Vector6<f64>, &f64) -> Vector6<f64>,
    {
        let halfstep = self.step_width * 0.5;
        let sixth_step_width = self.step_width / 6.0;

        let k1 = dxdt(&state, &self.time);

        let k2 = dxdt(&(state + k1 * halfstep), &(self.time + halfstep));

        let k3 = dxdt(&(state + k2 * halfstep), &(self.time + halfstep));

        let k4 = dxdt(
            &(state + self.step_width * k3),
            &(self.time + self.step_width),
        );

        state + sixth_step_width * (k1 + 2.0 * (k2 + k3) + k4)
    }

    pub fn propagate(&mut self) {
        let mut files: Vec<File> = Vec::new();
        let mut other_files: Vec<File> = Vec::new();

        let start_time = Instant::now();

        // storing and/or writing initial states for satellites
        for sat_num in 0..self.eoms.satellite.len() {
            if self.storeflag {
                // store first state
                let init_state = self.eoms.satellite[sat_num].state.data.0[0];
                self.eoms.satellite[sat_num]
                    .state_history
                    .push(init_state.to_vec());
            }
            if self.writeflag {
                // init file
                let filename = format!("outputs/output_{}.txt", self.eoms.satellite[sat_num].name);
                let file = File::create(&filename).expect("Failed to create file");
                files.push(file);

                // write first position
                writeln!(
                    files[sat_num],
                    "{}, {}, {}",
                    &self.eoms.satellite[sat_num].state[0],
                    &self.eoms.satellite[sat_num].state[1],
                    &self.eoms.satellite[sat_num].state[2]
                )
                .expect("Failed to write to file");
            }
        }
        // storing initial states for other bodies
        for other_num in 0..self.eoms.other_body.len() {
            if self.storeflag {
                // store first state
                let init_state = self.eoms.other_body[other_num].state.data.0[0];
                self.eoms.other_body[other_num]
                    .state_history
                    .push(init_state.to_vec());
            }

            if self.writeflag {
                // init file for other bodies
                let filename = format!(
                    "outputs/output_{}.txt",
                    self.eoms.other_body[other_num].name
                );
                let file = File::create(&filename).expect("Failed to create file for other body");
                other_files.push(file);

                // write first position
                writeln!(
                    other_files[other_num],
                    "{}, {}, {}",
                    &self.eoms.other_body[other_num].state[0],
                    &self.eoms.other_body[other_num].state[1],
                    &self.eoms.other_body[other_num].state[2]
                )
                .expect("Failed to write to other body's file");
            }

            let mut init_pos = Vector3::zeros();
            init_pos.fixed_rows_mut::<3usize>(0).copy_from(
                &self.eoms.other_body[other_num]
                    .state
                    .fixed_rows::<3usize>(0),
            );
            self.eoms.other_body[other_num]
                .pos_old
                .fixed_rows_mut::<3usize>(0)
                .copy_from(&init_pos);
        }

        // propagation ----------------------------------------------------------------------
        for _k in 0..self.maxsteps {
            // integrate for each satellite
            for sat_num in 0..self.eoms.satellite.len() {
                let current_state = self.eoms.satellite[sat_num].state.clone();

                let dxdt_fun = |state: &Vector6<f64>, time: &f64| -> Vector6<f64> {
                    self.eoms.dxdt(state, time, 9999)
                };

                let state_new = self.rk4_step(&dxdt_fun, &current_state);

                self.eoms.satellite[sat_num].state = state_new;
                // store state and time histories
                if self.storeflag {
                    let state_new = self.eoms.satellite[sat_num].state;
                    self.eoms.satellite[sat_num]
                        .state_history
                        .push(state_new.data.0[0].to_vec());
                    let time_new = self.time + self.step_width;
                    self.eoms.satellite[sat_num].time_history.push(time_new);
                }

                if self.writeflag {
                    writeln!(
                        files[sat_num],
                        "{}, {}, {}",
                        &self.eoms.satellite[sat_num].state[0],
                        &self.eoms.satellite[sat_num].state[1],
                        &self.eoms.satellite[sat_num].state[2]
                    )
                    .expect("Failed to write to file");
                    // println!("The new position is {:.4?}",&self.eoms.satellite.position);
                }
            }
            for other_num in 0..self.eoms.other_body.len() {
                // store old position before integration step
                let mut old_pos = Vector3::zeros();
                old_pos.fixed_rows_mut::<3usize>(0).copy_from(
                    &self.eoms.other_body[other_num]
                        .state
                        .fixed_rows::<3usize>(0),
                );
                self.eoms.other_body[other_num]
                    .pos_old
                    .fixed_rows_mut::<3usize>(0)
                    .copy_from(&old_pos);

                let current_state = self.eoms.other_body[other_num].state.clone();
                let dxdt_fun = |state: &Vector6<f64>, time: &f64| -> Vector6<f64> {
                    self.eoms
                        .dxdt2(state, time, self.eoms.other_body[other_num].id)
                };

                let state_new = self.rk4_step(&dxdt_fun, &current_state);

                self.eoms.other_body[other_num].state = state_new;

                // store state and time histories
                if self.storeflag {
                    let state_new = self.eoms.other_body[other_num].state;
                    self.eoms.other_body[other_num]
                        .state_history
                        .push(state_new.data.0[0].to_vec());
                    let time_new = self.time + self.step_width;
                    self.eoms.other_body[other_num].time_history.push(time_new);
                }
                if self.writeflag {
                    writeln!(
                        other_files[other_num],
                        "{}, {}, {}",
                        &self.eoms.other_body[other_num].state[0],
                        &self.eoms.other_body[other_num].state[1],
                        &self.eoms.other_body[other_num].state[2]
                    )
                    .expect("Failed to write to file");
                    // println!("The new position is {:.4?}",&self.eoms.satellite.position);
                }
            }
        }

        if self.timeflag == true {
            let end_time = Instant::now() - start_time;
            println!("Elapsed time: {:?}", end_time);
        }
    }
}
