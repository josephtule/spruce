mod math;
mod pointmass;
use math::*;
use pointmass::*;

fn main() {
    let re = 6_371.393e3;
    let position_eci = vec![2.0 * re, 2.0 * re, 0.0];
    let mu = 3.986e14;
    let acceleration_eci = gravity(&position_eci, mu);
    let acceleration_eci_mag = magnitude(&acceleration_eci);
    println!("{:?}", acceleration_eci_mag)
}
