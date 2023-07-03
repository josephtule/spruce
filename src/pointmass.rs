use crate::math::*;
pub fn gravity(position_eci: &[f64], mu: f64) -> Vec<f64> {
    let position_mag = magnitude(position_eci);
    let mut acceleration_eci = Vec::new();
    for ind in 0..3 {
        acceleration_eci.push(mu / position_mag.powi(3) * position_eci[ind]);
    }
    acceleration_eci
}

// pub fn rk4()
