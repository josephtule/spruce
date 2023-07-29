#[allow(unused_imports)]
use std::f64::consts::*;
mod attitude;
mod centralbody;
mod dynamical_system;
mod gravity;
mod math;
mod satbody;
// use math as ma; // use math with ma:: notation
// use math::*; // use math without math:: notation
use attitude::*;
use centralbody::*;
use dynamical_system::*;
use gravity::*;
use nalgebra::*;
use satbody::*;

fn main() {
    let mut earth = CentralBody {
        name: String::from("Earth"),
        mass: 5.97219e24,            // kg
        mu: 3.986004418000000e+14,   // kg.m^3/s^2
        equatorial_radius: 6378137., // m
        omega: 7.292115e-5,          // rad/s
        max_order: 0,
        max_deg: 0, // order >= degree
        c: vec![vec![]],
        s: vec![vec![]],
        eci2ecef: Matrix3::zeros(),
    };

    let grav_model = if earth.max_order > 1 && earth.max_deg > 0 {
        let filename = if earth.max_order > 361 {
            "egm2008_2159.txt"
        } else if earth.max_order > 121 {
            "egm2008_360.txt"
        } else {
            "egm2008_120.txt"
        };
        earth
            .read_sph_coefs(filename, earth.max_order, earth.max_deg)
            .expect("Could not read file");
        Gravity::sphharmonic(&earth)
    } else if earth.max_order > 1 && earth.max_deg == 0 {
        Gravity::j(&earth)
    } else {
        Gravity::spherical(&earth)
    };

    // let grav_model = Gravity {
    //     central_body: &earth,
    //     model: GravityModel::SphHarmonic(gravity::SphHarmonicGrav),
    // };

    // let grav_model = Gravity::sphharmonic(&earth);
    let time_0 = 0.;
    let v0 = 7.350157059479294e+03;
    let mut sat1 = Body {
        name: String::from("sat1"), // match struct name
        mass: 100.,                 //kg
        state: vector![earth.equatorial_radius + 1000e3, 0., 0., 0., v0, 0.,], // m, m/s
        propagate_flag: true,
        central_body: &earth,
        gravity: &grav_model,
        state_history: vec![],
        time_history: vec![time_0],
    };

    let mut sat2 = Body {
        name: String::from("sat2"), // match struct name
        mass: 100.,                 //kg
        state: vector![
            earth.equatorial_radius + 1000e3,
            0.,
            0.,
            0.,
            v0 * 0.5.sin(),
            v0 * 0.5.cos(),
        ], // m, m/s
        propagate_flag: true,
        central_body: &earth,
        gravity: &grav_model,
        state_history: vec![],
        time_history: vec![time_0],
    };

    let satellites = vec![&mut sat1, &mut sat2];

    let tspan = 3600. * 24. * 2.;
    let dt = 1.;

    #[allow(unused_mut)]
    #[allow(unused_assignments)]
    let mut n = (tspan / dt) as usize;

    let mut sys_temp = DynamicalSystem {
        maxsteps: n as usize,
        satellite: satellites,
        step_width: dt,
        time: 0.,
        central_body: &earth,
        writeflag: true,
        timeflag: true,
        storeflag: true,
    };

    sys_temp.propagate();

    let mut sat1_attitude = attitude::Attitude {
        euler: None, //
        quat: Some(Quaternions::new(vector![
            0.133026865470266,
            0.168722160571667,
            0.230813085987612,
            0.948979454430948
        ])),
        dcm: Matrix3::zeros(),
        main_rep: String::from("quat"),
    };
    sat1_attitude.gen_dcm();

    let mut sat2_attitude = attitude::Attitude {
        euler: Some(EulerAngles::new(
            vector![30. * PI / 180., 15. * PI / 180., 20. * PI / 180.],
            [3, 2, 1],
        )),
        quat: None,
        dcm: Matrix3::zeros(),
        main_rep: String::from("euler"),
    };
    sat2_attitude.gen_dcm();

    // print_smatrix(&sat1_attitude.dcm);
    // println!();
    // print_smatrix(&sat2_attitude.dcm);
    // sat2_attitude.update_others();

    // println!("{:?}", sys_temp.satellite[0].state_history);
}

use std::fmt::Display;
#[allow(dead_code)]
fn print_smatrix<T: Display, const R: usize, const C: usize>(mat: &SMatrix<T, R, C>) {
    print!("[");
    for i in 0..R {
        for j in 0..C {
            print!("{}, ", mat[(i, j)]);
        }
        if i == R - 1usize {
            print!("]");
        } else {
            println!();
        }
    }
    println!("");
}

#[allow(dead_code)]
fn print_dmatrix<T: Display>(mat: &DMatrix<T>) {
    let nrows = mat.nrows();
    let ncols = mat.ncols();

    print!("[");
    for i in 0..nrows {
        for j in 0..ncols {
            print!("{:16.4}, ", mat[(i, j)]);
        }
        if i == nrows - 1 {
            print!("]");
        } else {
            println!();
        }
    }
    println!("");
}
