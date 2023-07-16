#[allow(unused_imports)]

mod math;
mod pointmass;
mod centralbody;
// use math as ma; // use math with ma:: notation
use math::*; // use math without math:: notation
// use pointmass::*;
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
}
