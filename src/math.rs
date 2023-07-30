use nalgebra::*;
use std::fmt::Display;
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

#[allow(dead_code)]
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

#[allow(dead_code)]
pub fn normalize(vector: &Vec<f64>) -> Vec<f64> {
    let mut normalized_vector = vec![0.0; vector.len()];
    let vector_mag = magnitude(&vector);
    for i in 0..vector.len() {
        normalized_vector[i] = vector[i] / vector_mag;
    }
    normalized_vector
}

// TODO: adjust to allow vec<vec<>> AND vec<> input for the second value
#[allow(dead_code)]
pub fn matmul(a: &Vec<Vec<f64>>, b: &Vec<Vec<f64>>) -> Vec<Vec<f64>> {
    let a_rows = a.len();
    let a_cols = a[0].len();
    // let b_rows = b.len();
    let b_cols = b[0].len();

    let mut result = vec![vec![0.0; b_cols]; a_rows];

    for i in 0..a_rows {
        for j in 0..b_cols {
            for k in 0..a_cols {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }

    result
}

#[allow(dead_code)]
pub fn cross_operator(vector: Vec<f64>) -> Vec<Vec<f64>> {
    vec![
        vec![0., -vector[2], vector[1]],
        vec![vector[2], 0., -vector[0]],
        vec![-vector[1], vector[0], 0.],
    ]
}
#[allow(dead_code)]
pub fn colvec(vector: &Vec<f64>) -> Vec<Vec<f64>> {
    let mut newvec = vec![vec![0.]; vector.len()];
    for i in 0..vector.len() {
        newvec[i][0] = vector[i];
    }
    newvec
}

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
