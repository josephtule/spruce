use crate::math::*;

pub struct Attitude {
    pub euler: Option<EulerAngles>,
    pub quat: Option<Quaternions>,
    pub dcm: Vec<Vec<f64>>,
}

pub struct Quaternions {}

#[allow(dead_code)]
pub struct EulerAngles {
    pub angle: Vec<f64>,
    pub sequence: Vec<usize>,
}

#[allow(dead_code)]
impl EulerAngles {
    pub fn euler2dcm(&mut self) -> Vec<Vec<f64>> {
        // sequence "ABC" are multiplied as C*B*A
        // This method applies the Body sequence euler angle rather than the space sequence
        // The body sequence takes sequential rotations on the body axix to get a
        //     [BN] dcm (from inertial to body)
        // A space or inertial sequence takes sequential rotations on the inertial axis to get a
        //     [BN] dcm (from inertial to body)

        fn r1(t: f64) -> Vec<Vec<f64>> {
            vec![
                vec![1.0, 0.0, 0.0],
                vec![0.0, t.cos(), t.sin()],
                vec![0.0, -t.sin(), t.cos()],
            ]
        }

        fn r2(t: f64) -> Vec<Vec<f64>> {
            vec![
                vec![t.cos(), 0.0, -t.sin()],
                vec![0.0, 1.0, 0.0],
                vec![t.sin(), 0.0, t.cos()],
            ]
        }

        fn r3(t: f64) -> Vec<Vec<f64>> {
            vec![
                vec![t.cos(), t.sin(), 0.0],
                vec![-t.sin(), t.cos(), 0.0],
                vec![0.0, 0.0, 1.0],
            ]
        }
        let r_functions = [r1, r2, r3];
        let mut rot = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ]; // Identity matrix

        for i in (0..3).rev() {
            rot = matmul(&rot, &r_functions[self.sequence[i] - 1](self.angle[i]));
        }
        rot
    }
}
