use crate::math::*;

struct Body {
    name: String,
    mass: f64,
    position_init: [f64; 3],
    velocity_init: [f64; 3],
    propagate_flag: bool,

}

impl Body {
    fn new() -> Body {
        Body {
            name: String::from("satellite"), // match struct name
            // name: String::new(), // empty string
            mass: 100., //kg
            position_init: [100e3,100e3,100e3], // m
            velocity_init: [1.,1.,1.], // m/s
            propagate_flag: true,
        }
    }

    fn dxdt(&self, state: [f64;6]) -> [f64;6] { //consider moving to system struct, set as body
        //eoms
        let mut state_dot: [f64;6] = [0.;6];
        let r = magnitude(&state[0..3]);
        state_dot[0..3].copy_from_slice(&state[3..6]);
        state_dot[3] = - state[0] * 3.989e14 / r.powi(3);
        state_dot[4] = - state[1] * 3.989e14 / r.powi(3);
        state_dot[5] = - state[2] * 3.989e14 / r.powi(3);

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
