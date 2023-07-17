use crate::math::*;

pub struct Body<'a> {
    pub name: String,
    pub mass: f64,
    pub state: [f64; 6],
    pub propagate_flag: bool,
    pub central_body: &'a CentralBody,
}

pub struct CentralBody {
    pub name: String,
    pub mass: f64,
    pub mu: f64,
    pub equatorial_radius: f64,
}

#[allow(dead_code)]
impl<'a> Body<'a> {
    // pub fn new() -> Body<'a> {
    //     Body {
    //         name: String::from("satellite"), // match struct name
    //         // name: String::new(), // empty string
    //         mass: 100., //kg
    //         position: [100e3,100e3,100e3], // m
    //         velocity: [1.,1.,1.], // m/s
    //         propagate_flag: true,
    //         central_body: &CentralBody {
    //             name: String::from("Earth"),
    //             mass: 5.97219e24,
    //             mu: 3.986004418e14,
    //             equatorial_radius: 6.3781e6,
    //         },
    //     }
    // }

    #[allow(dead_code)]
    pub fn dxdt(&self, state: &[f64; 6], _time: &f64) -> [f64; 6] {
        //consider moving to system struct, set as body
        //eoms
        let mut state_dot: [f64; 6] = [0.; 6];
        let r = magnitude(&state[0..3]);
        state_dot[0..3].copy_from_slice(&state[3..6]);
        state_dot[3] = -state[0] * self.central_body.mu / r.powi(3);
        state_dot[4] = -state[1] * self.central_body.mu / r.powi(3);
        state_dot[5] = -state[2] * self.central_body.mu / r.powi(3);

        // set flags for perturbations
        // if self.sph_harmonics {
        //     let sph_harmonics_accel = get_sph_harmonics_accel(&state[0..3]);
        //     for i in 0..3 {
        //         state_dot[i+3] += sph_harmonics_accel[i];
        //     }
        // }
        state_dot
    }
}
