use crate::centralbody::*;
use crate::satbody::*;

use nalgebra::*;
use std::ops::AddAssign;

pub struct Gravity<'a> {
    pub satellite: Vec<&'a SatBody<'a>>,
    pub 
    pub central_body: &'a CentralBody,
    pub model: GravityModel,
    
}

#[allow(dead_code)]
impl<'a> Gravity<'a> {
    pub fn dxdt(&self, state: &Vector6<f64>, time: &f64) -> Vector6<f64> {
        match &self.model {
            GravityModel::Spherical(model) => model.calculate(&self.central_body, &state, &time),
            GravityModel::J(model) => model.calculate(&self.central_body, state, time),
            GravityModel::SphHarmonic(model) => model.calculate(&self.central_body, &state, &time), // ... other models
        }
    }
    pub fn spherical(central_body: &'a CentralBody) -> Self {
        Self {
            central_body,
            model: GravityModel::Spherical(SphericalGrav),
        }
    }
    pub fn j(central_body: &'a CentralBody) -> Self {
        Self {
            central_body,
            model: GravityModel::J(JGrav),
        }
    }

    pub fn sphharmonic(central_body: &'a CentralBody) -> Self {
        Self {
            central_body,
            model: GravityModel::SphHarmonic(SphHarmonicGrav),
        }
    }
}

#[allow(dead_code)]
#[derive(PartialEq)]
pub enum GravityModel {
    Spherical(SphericalGrav),
    J(JGrav),
    SphHarmonic(SphHarmonicGrav),
}

pub trait GravityCalculation {
    fn calculate(
        &self,
        central_body: &CentralBody,
        state: &Vector6<f64>,
        time: &f64,
    ) -> Vector6<f64>;
}
#[derive(PartialEq)]
pub struct SphericalGrav;
impl GravityCalculation for SphericalGrav {
    fn calculate(
        &self,
        central_body: &CentralBody,
        state: &Vector6<f64>,
        _time: &f64,
    ) -> Vector6<f64> {
        let mut state_dot = Vector6::zeros();

        // copy velocity into first three elements of dxdt
        state_dot
            .fixed_rows_mut::<3usize>(0)
            .copy_from(&state.fixed_rows::<3usize>(3));

        let mut x = Vector3::zeros();
        x.fixed_rows_mut::<3usize>(0)
            .copy_from(&state.fixed_rows::<3usize>(0));

        let r = x.norm();
        let muor3 = central_body.mu / r.powi(3);

        state_dot
            .fixed_rows_mut::<3usize>(3)
            .copy_from(&(state * -muor3).fixed_rows::<3usize>(0));

        state_dot
    }
}
#[derive(PartialEq)]
pub struct JGrav;
impl GravityCalculation for JGrav {
    fn calculate(
        &self,
        central_body: &CentralBody,
        state: &Vector6<f64>,
        _time: &f64,
    ) -> Vector6<f64> {
        let mut state_dot = Vector6::zeros();

        // copy velocity into first three elements of dxdt
        state_dot
            .fixed_rows_mut::<3usize>(0)
            .copy_from(&state.fixed_rows::<3usize>(3));

        let mut x = Vector3::zeros();
        x.fixed_rows_mut::<3usize>(0)
            .copy_from(&state.fixed_rows::<3usize>(0));
        let r = x.norm();
        let muor3 = central_body.mu / r.powi(3);

        state_dot
            .fixed_rows_mut::<3usize>(3)
            .copy_from(&(state * -muor3).fixed_rows::<3usize>(0));

        let j_vals = [
            1.08262668355e-3,  // j2
            -2.53265648533e-6, // j3
            -1.61962159137e-6, // j4
            -2.27296082869e-7, // j5
            5.40681239107e-7,  // j6
        ];

        let a_j2 = -3. / 2.
            * j_vals[0]
            * muor3
            * (central_body.equatorial_radius / r).powi(2)
            * vector![
                (1. - 5. * (state[2] / r).powi(2)) * state[0],
                (1. - 5. * (state[2] / r).powi(2)) * state[1],
                (3. - 5. * (state[2] / r).powi(2)) * state[2]
            ];
        state_dot.fixed_rows_mut::<3>(3).add_assign(&a_j2);
        state_dot
    }
}
#[derive(PartialEq)]
pub struct SphHarmonicGrav;
#[allow(non_snake_case)]
impl GravityCalculation for SphHarmonicGrav {
    fn calculate(
        &self,
        central_body: &CentralBody,
        state: &Vector6<f64>,
        time: &f64,
    ) -> Vector6<f64> {
        let ct = (central_body.omega * time).cos();
        let st = (central_body.omega * time).sin();

        let mut state_dot = Vector6::zeros();

        // copy velocity into first three elements of dxdt
        state_dot
            .fixed_rows_mut::<3usize>(0)
            .copy_from(&state.fixed_rows::<3usize>(3));

        let mut x = Vector3::zeros();
        x.fixed_rows_mut::<3usize>(0)
            .copy_from(&state.fixed_rows::<3usize>(0));

        let x = Vector3::new(x[0] * ct + x[1] * st, x[1] * ct - x[0] * st, x[2]);

        let maxord = central_body.max_order;
        let maxdeg = central_body.max_deg;
        let r = x.norm();

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
        let (P, scale_factor) = central_body.norm_legendre(phi, maxdeg, maxord);

        // start of calculations
        let r_ratio = central_body.equatorial_radius / r;
        let mut r_ratio_n = r_ratio;

        // init summation variables for gravity in spherical coordinates
        let mut dUdr_sum_n = 1.;
        let mut dUdphi_sum_n = 0.;
        let mut dUdlam_sum_n = 0.;

        // summation loop
        for n in 2..maxord + 1 {
            r_ratio_n = r_ratio_n * r_ratio;
            let mut dUdr_sum_m = 0.;
            let mut dUdphi_sum_m = 0.;
            let mut dUdlam_sum_m = 0.;
            let nf = n as f64;
            for m in 0..maxdeg + 1 {
                let mf = m as f64;

                dUdr_sum_m = dUdr_sum_m
                    + P[n][m]
                        * (central_body.c[n][m] * clam_vec[m] + central_body.s[n][m] * slam_vec[m]);
                dUdphi_sum_m = dUdphi_sum_m
                    + (P[n][m + 1] * scale_factor[n][m]
                        - x[2] / (x[0].powi(2) + x[1].powi(2)).sqrt() * mf * P[n][m])
                        * (central_body.c[n][m] * clam_vec[m] + central_body.s[n][m] * slam_vec[m]);
                dUdlam_sum_m = dUdlam_sum_m
                    + mf * P[n][m]
                        * (central_body.s[n][m] * clam_vec[m] - central_body.c[n][m] * slam_vec[m]);
            }

            dUdr_sum_n = dUdr_sum_n + dUdr_sum_m * r_ratio_n * (nf + 1.);
            dUdphi_sum_n = dUdphi_sum_n + dUdphi_sum_m * r_ratio_n;
            dUdlam_sum_n = dUdlam_sum_n + dUdlam_sum_m * r_ratio_n;
        }

        let muor = central_body.mu / r;
        let dUdr = -muor / r * dUdr_sum_n;
        let dUdphi = muor * dUdphi_sum_n;
        let dUdlam = muor * dUdlam_sum_n;

        let mut grav_sph = vector![
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
        ];

        // spherical harmonics gravity

        // let mut grav_sph = self.sphharmon_grav(&x_ecef); // acceleration due to gravity wrt to
        // ECI in ECEF coordinates (no need to
        // counteract rotational effects on acceleration)
        grav_sph = Vector3::new(
            grav_sph[0] * ct - grav_sph[1] * st,
            grav_sph[1] * ct + grav_sph[0] * st,
            grav_sph[2],
        );

        // grav_sph = eci2ecef.transpose() * grav_sph; // transformation from ecef to eci
        // https://spsweb.fltops.jpl.nasa.gov/portaldataops/mpg/MPG_Docs/Source%20Docs/gravity-SphericalHarmonics.pdf
        // page 5 to 7,
        // spherical harmonics calculations from sphharmon_grav takes in ecef, gets spherical
        // coordinates, then outputs gravity in ecef, use transform to get inertial gravity
        // (page 7)
        // or explained here https://space.stackexchange.com/questions/51806/difference-between-rotated-frame-and-rotating-frame
        state_dot.fixed_rows_mut::<3>(3).copy_from(&grav_sph);

        state_dot
    }
}
