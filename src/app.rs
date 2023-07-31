#[allow(unused_imports)]
use crate::centralbody::*;
#[allow(unused_imports)]
use crate::dynamical_system::*;
#[allow(unused_imports)]
use crate::eoms::*;
#[allow(unused_imports)]
use crate::otherbody::*;
#[allow(unused_imports)]
use crate::satbody::*;
use egui::*;
#[allow(unused_imports)]
use nalgebra::*;

pub struct MyApp {
    central_body: CentralBody,
    other_bodies: Vec<OtherBody>,
    sat_bodies: Vec<SatBody>,
    maxsteps: usize,
    step_width: f64,
    writeflag: bool,
    timeflag: bool,
    storeflag: bool,
    defaultdays: f64,
}

impl Default for MyApp {
    fn default() -> Self {
        Self {
            central_body: CentralBody::new(),
            other_bodies: vec![],
            sat_bodies: vec![],
            maxsteps: 0,
            step_width: 0.,
            writeflag: true,
            timeflag: true,
            storeflag: true,
            defaultdays: 2.,
        }
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        CentralPanel::default().show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("Default Orbit time (days):");
                ui.add(egui::DragValue::new(&mut self.defaultdays).speed(0.1));
                if ui.button("Set Default Scenario").clicked() {
                    self.default_scenario(self.defaultdays);
                }
            });
            // CentralBody UI
            ui.group(|ui| {
                ui.label("Central Body");
                ui.label("Name");
                ui.text_edit_singleline(&mut self.central_body.name);
                ui.label("Mass");
                ui.add(
                    egui::DragValue::new(&mut self.central_body.mass)
                        .speed(1e5)
                        .suffix(" kg"),
                );
                ui.label("Mu");
                ui.add(
                    egui::DragValue::new(&mut self.central_body.mu)
                        .speed(1e2)
                        .suffix(" km^3/s^2"),
                );
                ui.label("Equatorial Radius");
                ui.add(
                    egui::DragValue::new(&mut self.central_body.equatorial_radius)
                        .speed(1e2)
                        .suffix(" km"),
                );
                ui.label("Rotational Speed");
                ui.add(
                    egui::DragValue::new(&mut self.central_body.omega)
                        .speed(1e-7)
                        .suffix(" rad/s"),
                );
                ui.label("Spherical Harmonics");
                ui.add_sized(
                    [300., 20.],
                    egui::Slider::new(&mut self.central_body.max_order, 0..=2159)
                        .text("Max Order")
                        .logarithmic(true),
                );
                ui.add(
                    egui::Slider::new(&mut self.central_body.max_deg, 0..=2159)
                        .text("Max Degree")
                        .logarithmic(true),
                );
                if ui.button("Default Earth Values").clicked() {
                    self.central_body.name = String::from("Earth");
                    self.central_body.mass = 5.97219e24; // kg
                    self.central_body.mu = 3.986004418000000e+5; // km^3/s^2
                    self.central_body.equatorial_radius = 6378.137; // km
                    self.central_body.omega = 7.292115e-5;
                }
            });

            // OtherBodies UI
            ui.group(|ui| {
                ui.label("Other Bodies");
                let mut to_remove_other = Vec::new();
                for (index, other_body) in self.other_bodies.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.group(|ui| {
                            ui.label("Name");
                            ui.text_edit_singleline(&mut other_body.name);
                            ui.label("Mass");
                            ui.add(
                                egui::DragValue::new(&mut other_body.mass)
                                    .speed(1e4)
                                    .suffix(" kg"),
                            );
                            ui.label("Mu");
                            ui.add(
                                egui::DragValue::new(&mut other_body.mu)
                                    .speed(1e2)
                                    .suffix(" km^3/s^2"),
                            );
                            other_body.id = index;
                            ui.label("Body ID");
                            ui.add(egui::DragValue::new(&mut other_body.id).speed(1.0));
                            ui.horizontal(|ui| {
                                ui.label("Initial State:");
                                for state in other_body.state.iter_mut() {
                                    ui.add(egui::DragValue::new(state).speed(1e-3).suffix(" km"));
                                }
                            });
                        });
                        if ui.button("Remove").clicked() {
                            to_remove_other.push(index);
                        }
                    });
                }
                if ui.button("Add Other Body").clicked() {
                    self.other_bodies.push(OtherBody::new());
                }
                to_remove_other.reverse();
                for index in to_remove_other {
                    self.other_bodies.remove(index);
                }
            });

            // SatBodies UI
            ui.group(|ui| {
                ui.label("Satellite Bodies");
                let mut to_remove_other = Vec::new();
                for (index, sat_body) in self.sat_bodies.iter_mut().enumerate() {
                    ui.horizontal(|ui| {
                        ui.group(|ui| {
                            ui.label("Name");
                            ui.text_edit_singleline(&mut sat_body.name);
                            ui.label("Mass");
                            ui.add(
                                egui::DragValue::new(&mut sat_body.mass)
                                    .speed(0.1)
                                    .suffix(" kg"),
                            );
                            ui.horizontal(|ui| {
                                ui.label("Initial State:");
                                for state in sat_body.state.iter_mut() {
                                    ui.add(egui::DragValue::new(state).speed(1e-3).suffix(" km"));
                                }
                            });
                            if ui.button("Remove").clicked() {
                                to_remove_other.push(index);
                            }
                        });
                    });
                }
                if ui.button("Add Satellite Body").clicked() {
                    self.sat_bodies.push(SatBody::new());
                }
                to_remove_other.reverse();
                for index in to_remove_other {
                    self.other_bodies.remove(index);
                }
            });

            ui.group(|ui| {
                ui.label("Dynamical System Configuration");

                ui.horizontal(|ui| {
                    ui.label("Max Steps:");
                    ui.add(egui::DragValue::new(&mut self.maxsteps).speed(10));
                });

                ui.horizontal(|ui| {
                    ui.label("Step Width:");
                    ui.add(
                        egui::DragValue::new(&mut self.step_width)
                            .speed(0.01)
                            .suffix(" sec"),
                    );
                });

                ui.checkbox(&mut self.writeflag, "Write Flag");
                ui.checkbox(&mut self.timeflag, "Time Flag");
                ui.checkbox(&mut self.storeflag, "Store Flag");

                if ui.button("Run Propagation").clicked() {
                    // create spherical harmonic tables if needed
                    if self.central_body.max_order > 1 && self.central_body.max_deg > 0 {
                        let filename = if self.central_body.max_order > 361 {
                            "egm2008_2159.txt"
                        } else if self.central_body.max_order > 121 {
                            "egm2008_360.txt"
                        } else {
                            "egm2008_120.txt"
                        };
                        self.central_body
                            .read_sph_coefs(
                                filename,
                                self.central_body.max_order,
                                self.central_body.max_deg,
                            )
                            .expect("Could not read file");
                    }

                    let mut satellite_references: Vec<&mut SatBody> =
                        self.sat_bodies.iter_mut().collect();
                    let mut otherbody_references: Vec<&mut OtherBody> =
                        self.other_bodies.iter_mut().collect();

                    let mut gravity = if self.central_body.max_order > 1
                        && self.central_body.max_deg > 0
                    {
                        Eoms::sphharmonic(
                            &self.central_body,
                            &mut satellite_references,
                            &mut otherbody_references,
                        )
                    } else if self.central_body.max_order > 1 && self.central_body.max_deg == 0 {
                        Eoms::j(
                            &self.central_body,
                            &mut satellite_references,
                            &mut otherbody_references,
                        )
                    } else {
                        Eoms::spherical(
                            &self.central_body,
                            &mut satellite_references,
                            &mut otherbody_references,
                        )
                    };

                    let mut sys_temp = DynamicalSystem {
                        maxsteps: self.maxsteps as usize,
                        step_width: self.step_width,
                        time: 0.,
                        eoms: &mut gravity,
                        writeflag: self.writeflag,
                        timeflag: self.timeflag,
                        storeflag: self.storeflag,
                    };

                    sys_temp.propagate();
                    if sys_temp.writeflag && sys_temp.storeflag {
                        match sys_temp.writebinary() {
                            Ok(_) => println!("Writing succesful"),
                            Err(e) => println!("Error during writing: {}", e),
                        }
                        match sys_temp.writefiles() {
                            Ok(_) => println!("Writing succesful"),
                            Err(e) => println!("Error during writing: {}", e),
                        }
                    }
                    for sat in self.sat_bodies.iter_mut() {
                        if let Some(last_state) = sat.state_history.last() {
                            sat.state = Vector6::from_vec(last_state.clone());
                        }
                        sat.state_history = vec![];
                    }
                    for other in self.other_bodies.iter_mut() {
                        if let Some(last_state) = other.state_history.last() {
                            other.state = Vector6::from_vec(last_state.clone());
                        }
                        other.state_history = vec![];
                    }
                }
            });
        });
    }
}

impl MyApp {
    fn default_scenario(&mut self, days: f64) {
        let mut earth = CentralBody {
            name: String::from("Earth"),
            mass: 5.97219e24,            // kg
            mu: 3.986004418000000e+5,    // km^3/s^2
            equatorial_radius: 6378.137, // km
            omega: 7.292115e-5,          // rad/s
            max_order: 4, // [0,0] for spherical, [2,0] for J2, [2+,1+] for spherical harmonics
            max_deg: 4,   // order >= degree
            c: vec![vec![]],
            s: vec![vec![]],
            eci2ecef: Matrix3::zeros(),
        };
        if earth.max_order > 1 && earth.max_deg > 0 {
            let filename = if earth.max_order > 361 {
                "egm2008_2159.txt"
            } else if earth.max_order > 121 {
                "egm2008_360.txt"
            } else {
                "egm2008_120.txt"
            };
            earth
                .read_sph_coefs(filename, earth.max_order, earth.max_deg)
                .expect("Could not read file");
        }
        let time_0 = 0.;
        let moon_distance_from_earth = 384400.; // meters
        let moonv0 = (earth.mu / moon_distance_from_earth).sqrt();
        let moon1 = OtherBody {
            mass: 7.34e22,
            mu: 4.9048695e3,
            id: 1,
            pos_old: Vector3::zeros(),
            propagate_flag: true,
            state: vector![moon_distance_from_earth, 0., 0., 0., moonv0, 0.,], // Assuming moon starts on the x-axis and other velocities will be set elsewhere
            state_history: vec![],
            time_history: vec![],
            name: String::from("moon1"),
        };

        let moon2 = OtherBody {
            mass: 7.34e22,
            mu: 4.9048695e3,
            id: 2,
            pos_old: Vector3::zeros(),
            propagate_flag: true,
            state: vector![-moon_distance_from_earth, 0., 0., 0., -moonv0, 0.,], // Assuming moon starts on the x-axis and other velocities will be set elsewhere
            state_history: vec![],
            time_history: vec![],
            name: String::from("moon2"),
        };

        let v0 = 7.350157059479294;
        let moon_radius = 1.7371e3; // km
        let sat_altitude_above_moon = 100.; // 100 km
        let sat_distance_from_moon_center = moon_radius + sat_altitude_above_moon;
        let v0_sat = (moon1.mu / sat_distance_from_moon_center).sqrt();

        let sat1 = SatBody {
            name: String::from("sat1"),
            mass: 100.,
            state: vector![
                moon_distance_from_earth + sat_distance_from_moon_center,
                0.,
                0.,
                0.,
                moonv0 + v0_sat,
                0.,
            ],
            propagate_flag: true,
            state_history: vec![],
            time_history: vec![time_0],
        };
        let sat2 = SatBody {
            name: String::from("sat2"), // match struct name
            mass: 100.,                 //kg
            state: vector![
                earth.equatorial_radius + 1000.,
                0.,
                0.,
                0.,
                v0 * 0.5.sin(),
                v0 * 0.5.cos(),
            ], // m, m/s
            propagate_flag: true,
            state_history: vec![],
            time_history: vec![time_0],
        };

        let sat3 = SatBody {
            name: String::from("sat3"),
            mass: 100.,
            state: vector![
                -(moon_distance_from_earth + sat_distance_from_moon_center),
                0.,
                0.,
                0.,
                -(moonv0 + v0_sat),
                0.,
            ],
            propagate_flag: true,
            state_history: vec![],
            time_history: vec![time_0],
        };

        self.central_body = earth;
        self.other_bodies.clear();
        self.other_bodies.push(moon1);
        self.other_bodies.push(moon2);
        self.sat_bodies.clear();
        self.sat_bodies.push(sat1);
        self.sat_bodies.push(sat2);
        self.sat_bodies.push(sat3);
        let tspan = 3600. * 24. * days; // n days in seconds
        let dt = 10.;
        let n = (tspan / dt) as usize;
        self.step_width = dt;
        self.maxsteps = n;
    }
}
