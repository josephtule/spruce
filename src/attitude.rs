// use crate::math::*;
use nalgebra::*;
#[derive(Debug)]
pub struct Attitude {
    pub euler: Option<EulerAngles>,
    pub quat: Option<Quaternions>,
    pub dcm: Matrix3<f64>,
    pub main_rep: String,
}
#[derive(Debug)]
#[allow(dead_code)]
pub struct Quaternions {
    pub quaternion: Vector4<f64>,
    name: String,
}
#[derive(Debug)]
#[allow(dead_code)]
pub struct EulerAngles {
    pub angle: Vector3<f64>,
    pub sequence: [usize; 3],
    name: String,
}

#[allow(dead_code)]
#[allow(unused_variables)]
impl EulerAngles {
    pub fn new(angle: Vector3<f64>, sequence: [usize; 3]) -> Self {
        Self {
            angle,
            sequence,
            name: String::from("euler"),
        }
    }
    pub fn euler2dcm(&self) -> Matrix3<f64> {
        // sequence "ABC" are multiplied as C*B*A
        // This method applies the Body sequence euler angle rather than the space sequence
        // The body sequence takes sequential rotations on the body axix to get a
        //     [BN] dcm (from inertial to body)
        // A space or inertial sequence takes sequential rotations on the inertial axis to get a
        //     [BN] dcm (from inertial to body)

        fn r1(t: f64) -> Matrix3<f64> {
            matrix![
                1.0, 0.0, 0.0;
                0.0, t.cos(), t.sin();
                0.0, -t.sin(), t.cos();
            ]
        }

        fn r2(t: f64) -> Matrix3<f64> {
            matrix![
                t.cos(), 0.0, -t.sin();
                0.0, 1.0, 0.0;
                t.sin(), 0.0, t.cos();
            ]
        }

        fn r3(t: f64) -> Matrix3<f64> {
            matrix![
                t.cos(), t.sin(), 0.0;
                -t.sin(), t.cos(), 0.0;
                0.0, 0.0, 1.0;
            ]
        }
        let r_functions = [r1, r2, r3];
        let mut rot = Matrix3::identity(); // Identity matrix

        for i in (0..3).rev() {
            rot = rot * r_functions[self.sequence[i] - 1](self.angle[i]);
        }
        rot
    }
    pub fn dcm2euler(&self, dcm: Matrix3<f64>, new_sequence: [usize; 3]) /*-> Vector3<f64>*/
    {
        // dcm2euler here, depends on sequence
        // self.sequence = new_sequence;
    }
}

#[allow(dead_code)]
impl Quaternions {
    pub fn new(quaternion: Vector4<f64>) -> Self {
        Self {
            quaternion,
            name: String::from("quat"),
        }
    }

    pub fn quat2dcm(&self) -> Matrix3<f64> {
        let c1 = RowVector3::new(
            1. - 2. * self.quaternion[1].powi(2) - 2. * self.quaternion[2].powi(2),
            2. * (self.quaternion[0] * self.quaternion[1]
                + self.quaternion[2] * self.quaternion[3]),
            2. * (self.quaternion[0] * self.quaternion[2]
                - self.quaternion[1] * self.quaternion[3]),
        );
        let c2 = RowVector3::new(
            2. * (self.quaternion[0] * self.quaternion[1]
                - self.quaternion[2] * self.quaternion[3]),
            1. - 2. * self.quaternion[0].powi(2) - 2. * self.quaternion[2].powi(2),
            2. * (self.quaternion[1] * self.quaternion[2]
                + self.quaternion[0] * self.quaternion[3]),
        );

        let c3 = RowVector3::new(
            2. * (self.quaternion[0] * self.quaternion[2]
                + self.quaternion[1] * self.quaternion[3]),
            2. * (self.quaternion[1] * self.quaternion[2]
                - self.quaternion[0] * self.quaternion[3]),
            1. - 2. * self.quaternion[0].powi(2) - 2. * self.quaternion[1].powi(2),
        );

        Matrix3::from_rows(&[c1, c2, c3])
    }
    #[allow(dead_code)]
    #[allow(unused_variables)]
    pub fn dcm2quat(&self, dcm: Matrix3<f64>) /*-> Vector4<f64>*/
    {
        // dcm2quat code here, probably use shepards methods
    }
}
#[allow(dead_code)]
#[allow(unused_variables)]
impl Attitude {
    pub fn gen_dcm(&mut self) {
        if let Some(ref euler) = &mut self.euler {
            self.dcm = euler.euler2dcm();
        } else if let Some(ref quat) = &mut self.quat {
            self.dcm = quat.quat2dcm();
        }
    }
    pub fn update_others(&mut self) {
        if self.main_rep == "euler" {
            if let Some(ref quat) = &mut self.quat {
                // put dcm to quat here, self.quat.quaternion = self.quat.dcm2quat(self.dcm);
            }
        }
        if self.main_rep == "quat" {
            if let Some(ref euler) = &mut self.euler {
                // put dcm to quat here, self.euler.angle =
                // self.euler.dcm2euler(self.dcm,self.euler.sequence);
            }
        }
    }
    // TODO: add method to switch to another attitude representation
    pub fn switch_rep(&mut self, new_rep: String) {
        if new_rep == self.main_rep {
            //println!("already the main attitude representation");
            return;
        } else {
            self.main_rep = new_rep;
            if self.euler.is_none() {}
        }
    }
    // TODO: add a method to update all attitude represenations if one is updated
    // TODO: add a method to update a single representation
}
