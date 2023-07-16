#[allow(unused_imports)]
use std::fs::File;
use std::io::Write;
use std::time::Instant;

mod math;
mod pointmass;
mod dynamical_system;
// use math as ma; // use math with ma:: notation
// use math::*; // use math without math:: notation
use pointmass::*;
use dynamical_system::*;


fn main() {
    let start_time = Instant::now();
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
    let earth = CentralBody {
        name: String::from("Earth"),
        mass: 5.97219e24,
        mu: 3.986004418e14,
        equatorial_radius: 6.3781e6,
    };

    let mut sat1 = Body {
        name: String::from("sat1"), // match struct name
        mass: 100., //kg
        position: [earth.equatorial_radius + 1000e3,0.,0.], // m
        velocity: [0.,7.350157059479294e+03,0.], // m/s
        propagate_flag: true,
        central_body: &earth,
    };
    let mut sat2 = Body {
        name: String::from("sat2"), // match struct name
        mass: 100., //kg
        position: [earth.equatorial_radius + 1000e3,0.,0.], // m
        velocity: [0.,0.,7.350157059479294e+03], // m/s
        propagate_flag: true,
        central_body: &earth,
    };



    // println!("Initial Position: {:.4?}",sat1.position);
    // println!("Initial Velocity: {:.4?}",sat1.velocity);
    let n = 10000;
    let satellites = vec![&mut sat1, &mut sat2];
    let mut sys_temp = DynamicalSystem {
        mass: 6e24,
        equatorial_radius: 6789e3,
        mu: 3.987e14,
        satellite: satellites,
        step_width: 0.1,
        time: 0.,
        central_body: &earth,
    };
    // let mut file = File::create("output.txt").expect("Failed to create file");
    let mut files: Vec<File> = Vec::new();

    // loop through all satellites
    for sat_num in 0..sys_temp.satellite.len() {
        // propagate index
        let mut i = 1;
        
        // add file to files vector
        let filename = format!("output{}.txt",sat_num);
        let file = File::create(&filename).expect("Failed to create file");
        files.push(file);

        //write first line
        writeln!(files[sat_num],"{}, {}, {}",sys_temp.satellite[sat_num].position[0],sys_temp.satellite[sat_num].position[1],sys_temp.satellite[sat_num].position[2]).expect("Failed to write to file");

        // propagate for current satellite and write to file
        loop {
            sys_temp.rk4(sat_num);
            writeln!(files[sat_num],"{}, {}, {}",sys_temp.satellite[sat_num].position[0],sys_temp.satellite[sat_num].position[1],sys_temp.satellite[sat_num].position[2]).expect("Failed to write to file");
            // println!("The new position is {:.4?}",sys_temp.satellite.position);
            i += 1;
            if i > n {
                break;
            }
        }
    }

    let end_time = Instant::now() - start_time;
    println!("Elapsed time: {:?}",end_time);
}
