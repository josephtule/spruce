#[allow(unused_imports)]
use std::f64::consts::*;
mod app;
mod attitude;
mod centralbody;
mod dynamical_system;
mod eoms;
mod math;
mod orbitalelements;
mod otherbody;
mod satbody;
// use math as ma; // use math with ma:: notation
// use math::*; // use math without math:: notation
// use attitude::*;
#[allow(unused_imports)]
use app::*;
#[allow(unused_imports)]
use centralbody::*;
#[allow(unused_imports)]
use dynamical_system::*;
#[allow(unused_imports)]
use eframe::egui;
#[allow(unused_imports)]
use eoms::*;
#[allow(unused_imports)]
use nalgebra::*;
#[allow(unused_imports)]
use otherbody::*;
#[allow(unused_imports)]
use satbody::*;
#[allow(unused_imports)]
use std::time::Instant;

fn main() -> eframe::Result<()> {
    // let timestart = Instant::now();
    // let mut earth = CentralBody {
    //     name: String::from("Earth"),
    //     mass: 5.97219e24,            // kg
    //     mu: 3.986004418000000e+14,   // kg.m^3/s^2
    //     equatorial_radius: 6378137., // m
    //     omega: 7.292115e-5,          // rad/s
    //     max_order: 0, // [0,0] for spherical, [2,0] for J2, [2+,1+] for spherical harmonics
    //     max_deg: 0,   // order >= degree
    //     c: vec![vec![]],
    //     s: vec![vec![]],
    //     eci2ecef: Matrix3::zeros(),
    // };
    // if earth.max_order > 1 && earth.max_deg > 0 {
    //     let filename = if earth.max_order > 361 {
    //         "egm2008_2159.txt"
    //     } else if earth.max_order > 121 {
    //         "egm2008_360.txt"
    //     } else {
    //         "egm2008_120.txt"
    //     };
    //     earth
    //         .read_sph_coefs(filename, earth.max_order, earth.max_deg)
    //         .expect("Could not read file");
    // }
    // let time_0 = 0.;
    // let moon_distance_from_earth = 384400e3; // meters
    // let moonv0 = (earth.mu / moon_distance_from_earth).sqrt();
    // let mut moon1 = OtherBody {
    //     mass: 7.34e22,
    //     mu: 4.9048695e12,
    //     id: 1,
    //     pos_old: Vector3::zeros(),
    //     propagate_flag: true,
    //     state: vector![moon_distance_from_earth, 0., 0., 0., moonv0, 0.,], // Assuming moon starts on the x-axis and other velocities will be set elsewhere
    //     state_history: vec![],
    //     time_history: vec![],
    //     name: String::from("moon1"),
    // };
    //
    // let mut moon2 = OtherBody {
    //     mass: 7.34e22,
    //     mu: 4.9048695e12,
    //     id: 2,
    //     pos_old: Vector3::zeros(),
    //     propagate_flag: true,
    //     state: vector![-moon_distance_from_earth, 0., 0., 0., -moonv0, 0.,], // Assuming moon starts on the x-axis and other velocities will be set elsewhere
    //     state_history: vec![],
    //     time_history: vec![],
    //     name: String::from("moon2"),
    // };
    //
    // let v0 = 7.350157059479294e+03;
    // let moon_radius = 1.7371e6; // meters
    // let sat_altitude_above_moon = 100e3; // 100 km
    // let sat_distance_from_moon_center = moon_radius + sat_altitude_above_moon;
    // let v0_sat = (moon1.mu / sat_distance_from_moon_center).sqrt();
    //
    // let mut sat1 = SatBody {
    //     name: String::from("sat1"),
    //     mass: 100.,
    //     state: vector![
    //         moon_distance_from_earth + sat_distance_from_moon_center,
    //         0.,
    //         0.,
    //         0.,
    //         moonv0 + v0_sat,
    //         0.,
    //     ],
    //     propagate_flag: true,
    //     state_history: vec![],
    //     time_history: vec![time_0],
    // };
    // let mut sat2 = SatBody {
    //     name: String::from("sat2"), // match struct name
    //     mass: 100.,                 //kg
    //     state: vector![
    //         earth.equatorial_radius + 1000e3,
    //         0.,
    //         0.,
    //         0.,
    //         v0 * 0.5.sin(),
    //         v0 * 0.5.cos(),
    //     ], // m, m/s
    //     propagate_flag: true,
    //     state_history: vec![],
    //     time_history: vec![time_0],
    // };
    //
    // let mut sat3 = SatBody {
    //     name: String::from("sat3"),
    //     mass: 100.,
    //     state: vector![
    //         -(moon_distance_from_earth + sat_distance_from_moon_center),
    //         0.,
    //         0.,
    //         0.,
    //         -(moonv0 + v0_sat),
    //         0.,
    //     ],
    //     propagate_flag: true,
    //     state_history: vec![],
    //     time_history: vec![time_0],
    // };
    //
    // let mut satellite = vec![&mut sat1, &mut sat2, &mut sat3];
    // let mut otherbodies = vec![&mut moon1, &mut moon2];
    // let tspan = 3600. * 24. * 2.;
    // let dt = 5.;
    //
    // let mut gravity = if earth.max_order > 1 && earth.max_deg > 0 {
    //     Eoms::sphharmonic(&earth, &mut satellite, &mut otherbodies)
    // } else if earth.max_order > 1 && earth.max_deg == 0 {
    //     Eoms::j(&earth, &mut satellite, &mut otherbodies)
    // } else {
    //     Eoms::spherical(&earth, &mut satellite, &mut otherbodies)
    // };
    //
    // #[allow(unused_mut)]
    // #[allow(unused_assignments)]
    // let mut n = (tspan / dt) as usize;
    // n = 1e5 as usize;
    // let mut sys_temp = DynamicalSystem {
    //     maxsteps: n as usize,
    //     step_width: dt,
    //     time: 0.,
    //     eoms: &mut gravity,
    //     writeflag: true,
    //     timeflag: true,
    //     storeflag: true,
    // };
    // let endtime = Instant::now() - timestart;
    //
    // println!("Elapsed set-up time: {:?}", endtime);
    // // sys_temp.propagate();
    // sys_temp.propagate();
    // if sys_temp.writeflag && sys_temp.storeflag {
    //     match sys_temp.writebinary() {
    //         Ok(_) => println!("Writing succesful"),
    //         Err(e) => println!("Error during writing: {}", e),
    //     }
    //     match sys_temp.writefiles() {
    //         Ok(_) => println!("Writing succesful"),
    //         Err(e) => println!("Error during writing: {}", e),
    //     }
    // }

    // println!("{:?}", sys_temp.satellite[0].state_history);
    //
    // Create an instance of MyApp
    let my_app = MyApp::default();

    // Setup eframe context and integration
    let options = eframe::NativeOptions {
        initial_window_size: Some(egui::vec2(1400.0, 1000.0)),
        ..Default::default()
    };
    // let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "Orbit Propagator (use main.rs to get finer control)",
        options,
        Box::new(|_cc| Box::new(my_app)),
    )
}

// // attitude printing, not needed for now
//
// let mut sat1_attitude = attitude::Attitude {
//     euler: None, //
//     quat: Some(Quaternions::new(vector![
//         0.133026865470266,
//         0.168722160571667,
//         0.230813085987612,
//         0.948979454430948
//     ])),
//     dcm: Matrix3::zeros(),
//     main_rep: String::from("quat"),
// };
// sat1_attitude.gen_dcm();
//
// let mut sat2_attitude = attitude::Attitude {
//     euler: Some(EulerAngles::new(
//         vector![30. * PI / 180., 15. * PI / 180., 20. * PI / 180.],
//         [3, 2, 1],
//     )),
//     quat: None,
//     dcm: Matrix3::zeros(),
//     main_rep: String::from("euler"),
// };
// sat2_attitude.gen_dcm();
//
// // print_smatrix(&sat1_attitude.dcm);
// // println!();
// // print_smatrix(&sat2_attitude.dcm);
// // sat2_attitude.update_others();
//
