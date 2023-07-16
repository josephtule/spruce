use std::ops::Mul;

// pub fn magnitude(vector: &Vec<f64>) -> f64 {
//     let mut vector_mag: f64 = 0.;
//
//     for &element in vector {
//         vector_mag += element*element;
//     }
//     
//     vector_mag.sqrt()
// }

pub fn magnitude<T>(input: &[T]) -> f64
    where   
    T: Mul<Output = f64> + Copy,
{
    let square_sum = input
        .iter()
        .map(|x| *x * *x) // *x derefs to use value instead of ref
        .sum::<f64>();

    square_sum.sqrt()
}

pub fn normalize(vector: &Vec<f64>) -> Vec<f64> {
    let mut normalized_vector = vec![0.0; vector.len()];
    let vector_mag = magnitude(&vector);
    for i in 0..vector.len() {
        normalized_vector[i] = vector[i] / vector_mag;
    }
    normalized_vector
}
