use crate::math::*;

pub struct Attitude {
    pub euler: Option<EulerAngles>,
    pub quat: Option<Quaternions>,
    pub dcm: Vec<Vec<f64>>,
}

pub struct Quaternions {
    pub quaternion: Vec<f64>,
}

#[allow(dead_code)]
pub struct EulerAngles {
    pub angle: Vec<f64>,
    pub sequence: Vec<usize>,
}

#[allow(dead_code)]
impl EulerAngles {
    pub fn euler2dcm(&self, attitude: &mut Attitude) {
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
        attitude.dcm = rot;
    }
}

#[allow(dead_code)]
impl Quaternions {
    pub fn quat2dcm(&self, attitude: &mut Attitude) {
        let c1 = vec![
            1. - 2. * self.quaternion[1].powi(2) - 2. * self.quaternion[2].powi(2),
            2. * (self.quaternion[0] * self.quaternion[1]
                + self.quaternion[2] * self.quaternion[3]),
            2. * (self.quaternion[0] * self.quaternion[2]
                - self.quaternion[1] * self.quaternion[3]),
        ];
        let c2 = vec![
            2. * (self.quaternion[0] * self.quaternion[1]
                - self.quaternion[2] * self.quaternion[3]),
            1. - 2. * self.quaternion[0].powi(2) - 2. * self.quaternion[2].powi(2),
            2. * (self.quaternion[1] * self.quaternion[2]
                - self.quaternion[0] * self.quaternion[3]),
        ];

        let c3 = vec![
            2. * (self.quaternion[0] * self.quaternion[2]
                + self.quaternion[1] * self.quaternion[3]),
            2. * (self.quaternion[1] * self.quaternion[2]
                - self.quaternion[0] * self.quaternion[3]),
            1. - 2. * self.quaternion[0].powi(2) - 2. * self.quaternion[1].powi(2),
        ];

        attitude.dcm = vec![c1, c2, c3];
    }
}

impl Attitude {
    pub fn gen_dcm(&mut self) {
        if let Some(ref euler) = self.euler.take() {
            euler.euler2dcm(self)
        } else if let Some(ref quat) = self.quat.take() {
            quat.quat2dcm(self)
        }
    }

    // TODO: add a method to update all attitude represenations if one is updated
    // TODO: add a method to update a single representation
}
