pub fn magnitude(vector: &[f64]) -> f64 {
    let mut vector_mag = vector[0].powi(2) + vector[1].powi(2) + vector[2].powi(2);
    vector_mag = vector_mag.sqrt();
    vector_mag
}
