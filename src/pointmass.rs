use crate::math::*;
use core::f64::consts::PI;
pub struct Body<'a> {
    pub name: String,
    pub mass: f64,
    pub state: [f64; 6],
    pub propagate_flag: bool,
    pub central_body: &'a CentralBody,
}

#[allow(non_snake_case)]
pub struct CentralBody {
    pub name: String,
    pub mass: f64,
    pub mu: f64,
    pub equatorial_radius: f64,
    pub grav_flag: bool,
    pub max_deg: usize,
    pub c: Vec<Vec<f64>>,
    pub s: Vec<Vec<f64>>,
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
    #[allow(non_snake_case)]
    pub fn sphharmon_grav(&self, x: &[f64; 3]) -> [f64; 3] {
        let maxdeg = self.central_body.max_deg;
        let r = magnitude(x);

        let phi = (x[2] / r).asin();
        let lam = x[1].atan2(x[0]);
        let slam = lam.sin();
        let clam = lam.cos();

        let mut slam_vec = vec![0.; maxdeg + 1];
        let mut clam_vec = vec![0.; maxdeg + 1];

        slam_vec[0] = 0.;
        slam_vec[1] = slam;
        clam_vec[0] = 1.;
        clam_vec[1] = clam;

        for m in 2..maxdeg + 1 {
            slam_vec[m] = 2. * clam * slam_vec[m - 1] - slam_vec[m - 2];
            clam_vec[m] = 2. * clam * clam_vec[m - 1] - clam_vec[m - 2];
        }

        // generate normalized associated legendre polynomial matrices as well as scaling factors
        let (P, scale_factor) = self.central_body.norm_legendre(phi, maxdeg);

        // start of calculations
        let r_ratio = self.central_body.equatorial_radius / r;
        let mut r_ratio_n = r_ratio;

        // init summation variables for gravity in spherical coordinates
        let mut dUdr_sum_n = 1.;
        let mut dUdphi_sum_n = 0.;
        let mut dUdlam_sum_n = 0.;

        // summation loop
        for n in 2..maxdeg + 1 {
            // might need to change upper bound
            r_ratio_n = r_ratio_n * r_ratio;
            let mut dUdr_sum_m = 0.;
            let mut dUdphi_sum_m = 0.;
            let mut dUdlam_sum_m = 0.;
            let nf = n as f64;
            for m in 0..n {
                let mf = m as f64;

                dUdr_sum_m = dUdr_sum_m
                    + P[n][m]
                        * (self.central_body.c[n][m] * clam_vec[m]
                            + self.central_body.s[n][m] * slam_vec[m]);
                dUdphi_sum_m = dUdphi_sum_m
                    + (P[n][m + 1] * scale_factor[n][m]
                        - x[2] / (x[0].powi(2) + x[1].powi(2)).sqrt() * mf * P[n][m])
                        * (self.central_body.c[n][m] * clam_vec[m]
                            + self.central_body.s[n][m] * slam_vec[m]);
                dUdlam_sum_m = dUdlam_sum_m
                    + mf * P[n][m]
                        * (self.central_body.s[n][m] * clam_vec[m]
                            - self.central_body.c[n][m] * slam_vec[m]);
            }

            dUdr_sum_n = dUdr_sum_n + dUdr_sum_m * r_ratio_n * nf;
            dUdphi_sum_n = dUdphi_sum_n + dUdphi_sum_m * r_ratio_n;
            dUdlam_sum_n = dUdlam_sum_n + dUdlam_sum_m * r_ratio_n;
        }
        let dUdr = -self.central_body.mu / r.powi(2) * dUdr_sum_n;
        let dUdphi = self.central_body.mu / r * dUdphi_sum_n;
        let dUdlam = self.central_body.mu / r * dUdlam_sum_n;

        [
            ((1. / r) * dUdr
                - (x[2] / (r.powi(2) * (x[0].powi(2) + x[1].powi(2)).sqrt())) * dUdphi)
                * x[0]
                - (dUdlam / (x[0].powi(2) + x[1].powi(2))) * x[1],
            ((1. / r) * dUdr
                - (x[2] / (r.powi(2) * (x[0].powi(2) + x[1].powi(2)).sqrt())) * dUdphi)
                * x[1]
                + (dUdlam / (x[0].powi(2) + x[1].powi(2))) * x[0],
            (1. / r) * dUdr * x[2]
                + (((x[0].powi(2) + x[1].powi(2)).sqrt()) / (r.powi(2))) * dUdphi,
        ]
    }
}

impl CentralBody {
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    pub fn norm_legendre(&self, phi: f64, maxdeg: usize) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
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

        for n in 2..maxdeg + 3 {
            for m in 0..n + 1 {
                // compute the rest of the normalized associated legendre polynomials
                // using recursive relations as well as scale factors for normalization
                let nf = n as f64;
                let mf = m as f64;

                if n == m {
                    // sectoral
                    P[n][n] = (2. * nf + 1.).sqrt() / (2. * nf).sqrt() * sphi * P[n - 1][n - 1];
                    // scale_factor[n][n] = 0.; // not needed, setting 0 -> 0
                } else if m == 0 {
                    // zonal
                    P[n][m] = ((2. * nf + 1.).sqrt() / nf)
                        * ((2. * nf - 1.).sqrt() * cphi * P[n - 1][m]
                            - (nf - 1.) / (2. * nf - 3.).sqrt() * P[n - 2][m]);
                    scale_factor[n][m] = ((nf + 1.) * nf / 2.).sqrt();
                } else {
                    // tesseral
                    P[n][m] = (2. * nf + 1.).sqrt() / ((nf + mf).sqrt() * (nf - mf).sqrt())
                        * ((2. * nf - 1.).sqrt() * cphi * P[n - 1][m]
                            - (nf + mf - 1.).sqrt() * (nf - mf - 1.).sqrt()
                                / (2. * nf - 3.).sqrt()
                                * P[n - 2][m]);
                    scale_factor[n][m] = ((nf + mf + 1.) * (nf - mf)).sqrt();
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
        (P, scale_factor)
    }
}
