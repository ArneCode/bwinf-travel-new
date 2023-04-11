use std::{f64::consts::PI, fmt::Debug};

use crate::{angle_ok, Point};

#[derive(Clone)]
pub struct AngleOkList {
    data: Vec<bool>,
    n_points: usize,
}
impl AngleOkList {
    pub fn new(points: &Vec<Point>) -> Self {
        let n_points = points.len();
        let mut data = vec![false; n_points * n_points * n_points];
        for (i3, p3) in points.iter().enumerate() {
            for (i2, p2) in points.iter().enumerate() {
                for (i1, p1) in points.iter().enumerate() {
                    //19, 73, 7
                    let dir1 = p1.dir_to(p2);
                    let dir2 = p2.dir_to(p3);
                    let idx = Self::get_idx(n_points, (i1, i2, i3));
                    let angle = dir1.angle_to(&dir2);
                    data[idx] = angle_ok(angle);
                    if i3 == 7 && i2 == 73 && i1 == 19 {
                        println!("{} {} {} {}, {}", i1, i2, i3, angle * 180.0 / PI, data[idx]);
                    }
                }
            }
        }
        Self { n_points, data }
    }
    fn get_idx(n_points: usize, idx: (usize, usize, usize)) -> usize {
        n_points * n_points * idx.2 + n_points * idx.1 + idx.0
    }
    pub fn is_ok(&self, i1: usize, i2: usize, i3: usize) -> bool {
        self.data[Self::get_idx(self.n_points, (i1, i2, i3))]
    }
}
impl Debug for AngleOkList {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut s = String::new();
        for i3 in 0..self.n_points {
            for i2 in 0..self.n_points {
                for i1 in 0..self.n_points {
                    let idx = Self::get_idx(self.n_points, (i1, i2, i3));
                    s.push(if self.data[idx] { '1' } else { '0' });
                }
                s.push(' ');
            }
            s.push(' ');
        }
        write!(f, "{}", s)
    }
}
