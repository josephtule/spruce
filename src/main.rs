use std::f64::consts::*;
#[allow(unused_imports)]
use std::fs::File;
use std::io::Write;
use std::time::Instant;
mod attitude;
mod dynamical_system;
mod math;
mod pointmass;
// use math as ma; // use math with ma:: notation
// use math::*; // use math without math:: notation
use attitude::*;
use dynamical_system::*;
use pointmass::*;

fn main() {
    // let mut myvec = vec![1.;10];
    //
    // for i in 1..myvec.len() {
    //     myvec[i] = myvec[i] + myvec[i-1];
    // }
    //
    // let myvec_mag = magnitude(&myvec);
    // let myvec_norm = normalize(&myvec);
    //
    // println!("My vector is {:.4?}\n\
    //     its magnitude is {myvec_mag:.4}\n\
    //     and the normal vector is {:.4?}",myvec,myvec_norm);
    //
    let mut earth = CentralBody {
        name: String::from("Earth"),
        mass: 5.97219e24,
        mu: 3.986004418000000e+14,
        equatorial_radius: 6378137.,
        grav_flag: true,
        max_deg: 30,
        c: vec![vec![]],
        s: vec![vec![]],
    };
    let _read_result = earth.read_sph_coefs("egm2008data.txt", earth.max_deg, earth.max_deg);

    let mut sat1 = Body {
        name: String::from("sat1"), // match struct name
        mass: 100.,                 //kg
        state: [
            earth.equatorial_radius + 1000e3,
            0.,
            0.,
            0.,
            7.350157059479294e+03,
            0.,
        ], // m/s
        propagate_flag: true,
        central_body: &earth,
    };
    let mut sat2 = Body {
        name: String::from("sat2"), // match struct name
        mass: 100.,                 //kg
        state: [
            earth.equatorial_radius + 1000e3,
            0.,
            0.,
            0.,
            0.,
            7.350157059479294e+03,
        ], // m/s
        propagate_flag: true,
        central_body: &earth,
    };

    // println!("Initial Position: {:.4?}",sat1.position);
    // println!("Initial Velocity: {:.4?}",sat1.velocity);
    let n = 1000;
    let satellites = vec![&mut sat1, &mut sat2];
    let mut sys_temp = DynamicalSystem {
        satellite: satellites,
        step_width: 100.,
        time: 0.,
        central_body: &earth,
    };
    // let mut file = File::create("output.txt").expect("Failed to create file");
    let mut files: Vec<File> = Vec::new();
    let writefile = true;
    let start_time = Instant::now();
    // loop through all satellites
    for sat_num in 0..sys_temp.satellite.len() {
        // propagate index
        let mut i = 1;

        // add file to files vector
        let filename = format!("output{}.txt", sat_num);
        let file = File::create(&filename).expect("Failed to create file");
        files.push(file);

        //write first line
        if writefile {
            writeln!(
                files[sat_num],
                "{}, {}, {}",
                sys_temp.satellite[sat_num].state[0],
                sys_temp.satellite[sat_num].state[1],
                sys_temp.satellite[sat_num].state[2]
            )
            .expect("Failed to write to file");
            // println!("The new position is {:.4?}",sys_temp.satellite.position);
        }
        // propagate for current satellite and write to file
        loop {
            sys_temp.rk4(sat_num);
            if writefile {
                writeln!(
                    files[sat_num],
                    "{}, {}, {}",
                    sys_temp.satellite[sat_num].state[0],
                    sys_temp.satellite[sat_num].state[1],
                    sys_temp.satellite[sat_num].state[2]
                )
                .expect("Failed to write to file");
                // println!("The new position is {:.4?}",sys_temp.satellite.position);
            }

            i += 1;
            if i > n {
                break;
            }
        }
    }

    let end_time = Instant::now() - start_time;
    println!("Elapsed time: {:?}", end_time);

    let mut sat1_attitude = attitude::Attitude {
        euler: Some(EulerAngles {
            angle: vec![30. * PI / 180., 15. * PI / 180., 20. * PI / 180.],
            sequence: vec![3, 2, 1],
            dcm: vec![vec![]],
        }),
        quat: None,
    };

    let mut externaldcm = vec![vec![]];

    if let Some(euler) = &mut sat1_attitude.euler {
        euler.euler2dcm();
        externaldcm = euler.dcm.clone();
    };

    for i in 0..externaldcm.len() {
        println!("{:?}", externaldcm[i]);
    }
}
