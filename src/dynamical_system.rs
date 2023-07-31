// use crate::math::*;
use crate::eoms::*;
// use crate::otherbody::*;
// use crate::satbody::*;
// use matfile::{MatFile, NumericData};
use bincode::serialize_into;
use nalgebra::*;

// use ndarray::Array2;
use std::error::Error;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::time::Instant;
// use nalgebra::*;
pub struct DynamicalSystem<'a> {
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
            }
        }

        if self.timeflag == true {
            let end_time = Instant::now() - start_time;
            println!("Elapsed propagation time: {:?}", end_time);
        }
    }

    pub fn writefiles(&self) -> Result<(), Box<dyn Error>> {
        // Ensure the directory exists
        std::fs::create_dir_all("outputs/txt")?;
        let mut files: Vec<BufWriter<File>> = Vec::new();
        let mut other_files: Vec<BufWriter<File>> = Vec::new();

        let start_time = Instant::now();

        // Creating files
        for satellite in self.eoms.satellite.iter() {
            let filename = format!("outputs/txt/output_{}.txt", satellite.name);
            let file = BufWriter::new(File::create(&filename)?);
            files.push(file);
        }

        for body in self.eoms.other_body.iter() {
            let filename = format!("outputs/txt/output_other_{}.txt", body.name);
            let file = BufWriter::new(File::create(&filename)?);
            other_files.push(file);
        }

        // Writing to files
        for k in 0..self.maxsteps {
            for (sat_num, satellite) in self.eoms.satellite.iter().enumerate() {
                writeln!(
                    files[sat_num],
                    "{:?}, {:?}, {:?}",
                    satellite.state_history[k][0],
                    satellite.state_history[k][1],
                    satellite.state_history[k][2]
                )?;
            }

            for (other_num, body) in self.eoms.other_body.iter().enumerate() {
                writeln!(
                    other_files[other_num],
                    "{}, {}, {}",
                    body.state_history[k][0], body.state_history[k][1], body.state_history[k][2]
                )?;
            }
        }

        for writer in &mut files {
            writer.flush()?;
        }

        for writer in &mut other_files {
            writer.flush()?;
        }

        let end_time = Instant::now() - start_time;
        println!("Elapsed writing time (.txt): {:?}", end_time);
        Ok(())
    }

    pub fn writebinary(&self) -> Result<(), Box<dyn Error>> {
        // Ensure the directory exists
        std::fs::create_dir_all("outputs/bin")?;
        let mut files: Vec<BufWriter<File>> = Vec::new();
        let mut other_files: Vec<BufWriter<File>> = Vec::new();

        let start_time = Instant::now();

        // Creating files
        for satellite in self.eoms.satellite.iter() {
            let filename = format!("outputs/bin/output_{}.bin", satellite.name);
            let file = BufWriter::new(File::create(&filename)?);
            files.push(file);
        }

        for body in self.eoms.other_body.iter() {
            let filename = format!("outputs/bin/output_other_{}.bin", body.name);
            let file = BufWriter::new(File::create(&filename)?);
            other_files.push(file);
        }

        // Writing to files
        for k in 0..self.maxsteps {
            for (sat_num, satellite) in self.eoms.satellite.iter().enumerate() {
                let data = [
                    satellite.state_history[k][0],
                    satellite.state_history[k][1],
                    satellite.state_history[k][2],
                ];
                serialize_into(&mut files[sat_num], &data)?;
            }

            for (other_num, body) in self.eoms.other_body.iter().enumerate() {
                let data = [
                    body.state_history[k][0],
                    body.state_history[k][1],
                    body.state_history[k][2],
                ];
                serialize_into(&mut other_files[other_num], &data)?;
            }
        }

        for writer in &mut files {
            writer.flush()?;
        }

        for writer in &mut other_files {
            writer.flush()?;
        }

        let end_time = Instant::now() - start_time;
        println!("Elapsed writing time (.bin): {:?}", end_time);
        Ok(())
    }

    // pub fn writemat(&self) -> Result<(), Box<dyn Error>> {
    //     let start_time = Instant::now();
    //
    //     // Create a new MAT file for satellites
    //     let mut sat_file = MatFile::new("outputs/satellites.mat")?;
    //
    //     for satellite in &self.eoms.satellite {
    //         // Convert satellite state history to 2D ndarray (assuming state_history is Vec<Vec<f64>>)
    //         let matrix: Array2<f64> = Array2::from_shape_vec(
    //             (self.maxsteps, 3),
    //             satellite
    //                 .state_history
    //                 .clone()
    //                 .into_iter()
    //                 .flatten()
    //                 .collect(),
    //         )?;
    //
    //         let data = NumericData::Double {
    //             real: matrix.into_raw_vec(),
    //             imag: None,
    //         };
    //
    //         sat_file.write_numeric(&satellite.name, data)?;
    //     }
    //
    //     // Create a new MAT file for other bodies
    //     let mut body_file = MatFile::new("outputs/other_bodies.mat")?;
    //
    //     for body in &self.eoms.other_body {
    //         let matrix: Array2<f64> = Array2::from_shape_vec(
    //             (self.maxsteps, 3),
    //             body.state_history.clone().into_iter().flatten().collect(),
    //         )?;
    //
    //         let data = NumericData::Double {
    //             real: matrix.into_raw_vec(),
    //             imag: None,
    //         };
    //
    //         body_file.write_numeric(&body.name, data)?;
    //     }
    //
    //     let end_time = Instant::now() - start_time;
    //     println!("Elapsed writing time: {:?}", end_time);
    //     Ok(())
    // }
}
