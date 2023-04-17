use std::{f64::consts::PI, fmt::Debug};

use crate::{Point, Args};

// Die Richtung von einem Punkt zu einem Anderen
#[derive(Debug)]
pub struct Dir(pub f64, pub f64);
impl Dir {
    // Gibt den Winkel zwischen zwei Richtungen zurück
    pub fn angle_to(&self, other: &Dir) -> f64 {
        let dot = self.0 * other.0 + self.1 * other.1;
        f64::acos(dot / (self.len() * other.len())) * 180.0 / PI
    }
    // Gibt die Länge der Richtung zurück
    fn len(&self) -> f64 {
        f64::sqrt(self.0 * self.0 + self.1 * self.1)
    }
}
// Gibt zurück ob der Winkel in Grad zwischen 0° und 90° oder 270° und 360° ist
pub fn angle_ok(a: f64) -> bool {
    //schneidet ab der 3. Nachkommastelle ab
    //um bei 90° und 270° Fehler zu vermeiden
    let a = (a * 1000.0).round() / 1000.0;
    a <= 90.0 || a >= 270.0
}
#[derive(Clone)]
pub struct AngleOkList {
    /// data ist ein 1D Array der Größe n_points * n_points * n_points
    /// hier wird gespeichert ob der Winkel zwischen den Punkten i1, i2, i3
    /// ok ist
    data: Vec<bool>,
    n_points: usize,
}
impl AngleOkList {
    // Speichert, ob drei Punkte einen Winkel haben der in Ordnung ist
    pub fn new(points: &Vec<Point>, args: &Args) -> Self {
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
                    data[idx] = angle_ok(angle) || args.dont_check_angle;
                }
            }
        }
        Self { n_points, data }
    }
    // Gibt den Index im 1D Array zurück
    fn get_idx(n_points: usize, idx: (usize, usize, usize)) -> usize {
        n_points * n_points * idx.2 + n_points * idx.1 + idx.0
    }
    // Gibt zurück ob der Winkel zwischen den Punkten i1, i2, i3 ok ist
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
