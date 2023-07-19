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
    pub grav_flag: bool.
}

#[allow(dead_code)]
impl<'a> Body<'a> {
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

    #[allow(dead_code)]
    pub fn sphharmon_grav(&self, state: &[f64; 6], _time: &f64) -> [f64; 6] {
        // code will generate the acceleration
    }
}

impl CentralBody {
    #[allow(dead_code)]
    pub fn norm_legendre(&self, phi: f64, maxdeg: i32) {
        // generate legendre polynomials based on degree and phi (geocentric latitude)
        let mut P = vec![vec![0.; maxdeg + 3]; maxdeg + 3];
        let mut scale_factor = vec![vec![0.; maxdeg + 3]; maxdeg + 3];
    
        let cphi = (PI / 2. - phi).cos();
        let sphi = (PI / 2. - phi).sin();
    
        // seeds for normalized recursive formula
        P[0][0] = 1.; // n = 0, m = 0;
        P[1][0] = 3.0_f64.sqrt() * cphi;
        P[1][1] = 3.0_f64.sqrt() * sphi;
        scale_factor[0][0] = 0.;
        scale_factor[1][0] = 1.;
        scale_factor[1][1] = 0.;
    
        for n in 2..maxdeg+3 {
            for m in 0..n+1 {
                // compute the rest of the normalized associated legendre polynomials
                // using recursive relations as well as scale factors for normalization
                let nf = n as f64;
                let mf = m as f64;
                
                if n == m { // sectoral
                    P[n][n] = (2. * nf + 1.).sqrt() / (2. * nf).sqrt() * sphi * P[n-1][n-1];
                    // scale_factor[n][n] = 0.; // not needed, setting 0 -> 0    
                } else if m == 0 { // zonal
                    P[n][m] = ((2. * nf + 1.).sqrt() / nf) * ((2. * nf - 1.).sqrt() * cphi 
                        * P[n-1][m] - (nf - 1.) / (2. * nf - 3.).sqrt() * P[n-2][m]);
                    scale_factor[n][m] = ((nf + 1.) * nf / 2.).sqrt();
                } else { // tesseral
                    P[n][m] = (2. * nf + 1.).sqrt() / ((nf + mf).sqrt() *(nf - mf).sqrt()) * 
                        ((2. * nf - 1.).sqrt() * cphi * P[n-1][m] - (nf + mf - 1.).sqrt() * 
                        (nf - mf - 1.).sqrt() / (2. * nf - 3.).sqrt() * P[n-2][m]);
                    scale_factor[n][m] = ((nf + mf + 1.)*(nf - mf)).sqrt();    
                }
    
            }
        }
    
        /* println!("P matrix");
        for i in 0..P.len() {
            println!("{:.6?}", P[i]);
        }
        println!("\nScaling Factor Matrix");
        for i in 0..P.len() {
            println!("{:.6?}", scale_factor[i]);
        } */
    }

}
