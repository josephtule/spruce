use crate::math::*;

pub struct Attitude {
    pub euler: Option<EulerAngles>,
    pub quat: Option<Quaternions>,
}

pub struct Quaternions {}

#[allow(dead_code)]
pub struct EulerAngles {
    pub angle1: f64,
    pub angle2: f64,
    pub angle3: f64,
    pub sequence: String,
    pub dcm: Vec<Vec<f64>>,
}

#[allow(dead_code)]
impl EulerAngles {
    pub fn euler2dcm(&self) -> Vec<Vec<f64>> {
        // sequence "ABC" are multiplied as C*B*A
        // This method applies the Body sequence euler angle rather than the space sequence
        // The body sequence takes sequential rotations on the body axix to get a
        //     [BN] dcm (from inertial to body)
        // A space or inertial sequence takes sequential rotations on the inertial axis to get a
        //     [BN] dcm (from inertial to body)

        // rotate about x axis
        fn rot1(theta: f64) -> Vec<Vec<f64>> {
            vec![
                vec![1., 0., 0.],
                vec![0., theta.cos(), theta.sin()],
                vec![0., -theta.sin(), theta.cos()],
            ]
        }
        // rotate about y axis
        fn rot2(theta: f64) -> Vec<Vec<f64>> {
            vec![
                vec![theta.cos(), 0., -theta.sin()],
                vec![0., 1., 0.],
                vec![theta.sin(), 0., theta.cos()],
            ]
        }
        // rotate about z axis
        fn rot3(theta: f64) -> Vec<Vec<f64>> {
            vec![
                vec![theta.cos(), theta.sin(), 0.],
                vec![-theta.sin(), theta.cos(), 0.],
                vec![0., 0., 1.],
            ]
        }

        // match with correct sequence:
        match &self.sequence[..] {
            // 6 asymmetric sets
            "123" => matmul(
                &rot3(self.angle3),
                &matmul(&rot2(self.angle2), &rot1(self.angle1)),
            ),
            "132" => matmul(
                &rot2(self.angle3),
                &matmul(&rot3(self.angle2), &rot1(self.angle1)),
            ),
            "231" => matmul(
                &rot1(self.angle3),
                &matmul(&rot3(self.angle2), &rot2(self.angle1)),
            ),
            "213" => matmul(
                &rot3(self.angle3),
                &matmul(&rot1(self.angle2), &rot2(self.angle1)),
            ),
            "312" => matmul(
                &rot2(self.angle3),
                &matmul(&rot1(self.angle2), &rot3(self.angle1)),
            ),
            "321" => matmul(
                &rot1(self.angle3),
                &matmul(&rot2(self.angle2), &rot3(self.angle1)),
            ),

            // 6 symmetric sets
            "121" => matmul(
                &rot1(self.angle3),
                &matmul(&rot2(self.angle2), &rot1(self.angle1)),
            ),
            "131" => matmul(
                &rot1(self.angle3),
                &matmul(&rot3(self.angle2), &rot2(self.angle1)),
            ),
            "232" => matmul(
                &rot2(self.angle3),
                &matmul(&rot3(self.angle2), &rot2(self.angle1)),
            ),
            "212" => matmul(
                &rot2(self.angle3),
                &matmul(&rot1(self.angle2), &rot2(self.angle1)),
            ),
            "313" => matmul(
                &rot3(self.angle3),
                &matmul(&rot1(self.angle2), &rot3(self.angle1)),
            ),
            "323" => matmul(
                &rot3(self.angle3),
                &matmul(&rot2(self.angle2), &rot3(self.angle1)),
            ),
            _ => panic!("Invalid Sequence"),
        }
    }
}
