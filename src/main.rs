#[allow(unused_imports)]

mod math;
mod pointmass;
mod dynamical_system;
// use math as ma; // use math with ma:: notation
use math::*; // use math without math:: notation
use pointmass::*;
// use centralbody::*;


fn main() {
    let mut myvec = vec![1.;10];

    for i in 1..myvec.len() {
        myvec[i] = myvec[i] + myvec[i-1];
    }

    let myvec_mag = magnitude(&myvec);
    let myvec_norm = normalize(&myvec);

    println!("My vector is {:.4?}\n\
        its magnitude is {myvec_mag:.4}\n\
        and the normal vector is {:.4?}",myvec,myvec_norm);
    let sat1 = Body {
        name: String::from("sat1"), // match struct name
        mass: 100., //kg
        position: [100e3,100e3,100e3], // m
        velocity: [1.,1.,1.], // m/s
        propagate_flag: true,
    };

    println!("Initial Position: {:.4?}",sat1.position);
    println!("Initial Velocity: {:.4?}",sat1.velocity);

}
