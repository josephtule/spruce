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
pub fn print_smatrix<T: Display, const R: usize, const C: usize>(mat: &SMatrix<T, R, C>) {
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
pub fn print_dmatrix<T: Display>(mat: &DMatrix<T>) {
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

#[allow(dead_code)]
pub fn print_vec_of_vecs<T: std::fmt::Display>(vecs: &Vec<Vec<T>>) {
    for vec in vecs {
        for item in vec.iter() {
            print!("{} ", item);
        }
        println!();
    }
}

#[allow(non_snake_case, dead_code)]
fn state2coes(state: Vector6<f64>, mu: f64) -> (f64,f64,f64,f64,f64,f64,f64) {
    let mut r_vec = Vector3::zeros();
    let mut v_vec = Vector3::zeros();
    r_vec.fixed_rows_mut::<3usize>(0).copy_from(&state.fixed_rows::<3usize>(0)); // consistently faster
    v_vec.fixed_rows_mut::<3usize>(0).copy_from(&state.fixed_rows::<3usize>(3));

    let h_vec = r_vec.cross(&v_vec);

    let h_mag: f64 = h_vec.norm();
    let v_mag = v_vec.norm();
    let r_mag = r_vec.norm();
    let n_vec = Vector3::new(0.,0.,1.).cross(&h_vec);
    let n_mag = n_vec.norm();
    let e_vec = ((v_mag.powi(2) - mu / r_mag) * r_vec - r_vec.dot(&v_vec) * v_vec) / mu;
    let e = e_vec.norm();

    let energy = v_mag.powi(2) / 2. - mu / r_mag;
    let a: f64;
    let p: f64;
    if e != 1.0 {
        a = - mu / 2. / energy;
        p = a*(1. - e.powi(2));
    } else {
        p = h_mag.powi(2) / mu;
        a = f64::INFINITY;
    }


    let i = (h_vec[2]/h_mag).acos();
    let mut RAAN = (n_vec[1]/n_mag).acos();
    if n_vec[1] < 0. {
        RAAN = 2.* PI - RAAN;
    }
    if RAAN.is_nan() {
        RAAN = 0.;
    }
    let mut AOP = (n_vec.dot(&e_vec) / (n_mag * e)).acos();
    if e_vec[2] < 0. {
        AOP = 2. * PI - AOP;
    }
    if AOP.is_nan() {
        AOP = 0.;
    }
    let mut TA = (e_vec.dot(&r_vec) /(e * r_mag)).acos();
    if r_vec.dot(&v_vec) < 0. {
        TA = 2. * PI - TA;
        println!("{}",TA);
    }
    (a, e, p, i, RAAN, AOP, TA)
}
