use nalgebra::*;
use std::f64::consts::PI;
use std::fs::File;
use std::io::{self, BufRead, BufReader};
#[derive(Clone)]
#[allow(non_snake_case)]
pub struct CentralBody {
    pub name: String,
    pub mass: f64,
    pub mu: f64,
    pub equatorial_radius: f64,
    pub max_deg: usize,
    pub max_order: usize,
    pub c: Vec<Vec<f64>>,
    pub s: Vec<Vec<f64>>,
    pub eci2ecef: Matrix3<f64>, // simple transformation from eci2ecef
    pub omega: f64,
}

#[allow(dead_code)]
impl CentralBody {
    pub fn gen_eci2ecef(&self, time: &f64) -> Matrix3<f64> {
        matrix![(self.omega * time).cos(),  (self.omega * time).sin(), 0.;
                -(self.omega * time).sin(), (self.omega * time).cos(), 0.;
                0., 0., 1.]
    }
    #[allow(non_snake_case)]
    #[allow(dead_code)]
    pub fn norm_legendre(
        &self,
        phi: f64,
        maxdeg: usize,
        maxord: usize,
    ) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        // generate legendre polynomials based on degree and phi (geocentric latitude)
        let mut P = vec![vec![0.; maxdeg + 3]; maxord + 3];
        let mut scale_factor = vec![vec![0.; maxdeg + 3]; maxord + 3];

        let cphi = (PI / 2. - phi).cos();
        let sphi = (PI / 2. - phi).sin();

        // seeds for normalized recursive formula
        P[0][0] = 1.; // n = 0, m = 0;
        P[1][0] = 3.0_f64.sqrt() * cphi;
        P[1][1] = 3.0_f64.sqrt() * sphi;
        scale_factor[0][0] = 0.;
        scale_factor[1][0] = 1.;
        scale_factor[1][1] = 0.;

        for n in 2..maxord + 3 {
            for m in 0..maxdeg + 1 {
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

    pub fn read_sph_coefs(
        &mut self,
        file_path: &str,
        max_n: usize,
        max_m: usize,
    ) -> io::Result<()> {
        let parse_egm_line =
            |line: &str| -> Result<(usize, usize, f64, f64), std::num::ParseFloatError> {
                let parts: Vec<&str> = line.split_whitespace().collect();

                let n = parts[0].parse::<usize>().unwrap();
                let m = parts[1].parse::<usize>().unwrap();

                // Convert D notation to E notation for parsing in Rust
                let c_str = parts[2].replace("D", "E");
                let s_str = parts[3].replace("D", "E");

                let c = c_str.parse::<f64>()?;
                let s = s_str.parse::<f64>()?;

                Ok((n, m, c, s))
            };

        let file = File::open(file_path)?;
        let reader = BufReader::new(file);

        let mut current_n = 0;
        self.c = vec![vec![0.; max_m + 1]; max_n + 1];
        self.s = vec![vec![0.; max_m + 1]; max_n + 1];
        for line in reader.lines() {
            if let Ok(valid_line) = line {
                let (n, m, c, s) = parse_egm_line(&valid_line).unwrap_or((0, 0, 0.0, 0.0));

                if n != current_n && n <= max_n {
                    current_n = n;
                }

                if n <= max_n && m <= max_m {
                    self.c[n][m] = c;
                    self.s[n][m] = s;
                }

                // Stop reading if we have met the max n and m
                if n == max_n && m == max_m {
                    break;
                }
            }
        }

        Ok(())
    }

    pub fn new() -> Self {
        CentralBody {
            name: String::from(""),
            mass: 0.,              // kg
            mu: 0.,                // kg.m^3/s^2
            equatorial_radius: 0., // m
            omega: 0.,             // rad/s
            max_order: 0, // [0,0] for spherical, [2,0] for J2, [2+,1+] for spherical harmonics
            max_deg: 0,   // order >= degree
            c: vec![vec![]],
            s: vec![vec![]],
            eci2ecef: Matrix3::zeros(),
        }
    }
}
