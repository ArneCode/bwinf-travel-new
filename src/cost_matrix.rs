use std::fmt;

use crate::Point;

#[derive(Clone, Debug)]
pub struct CostMatrix {
    data: Vec<Option<f64>>,
    pub size: usize,
}
impl fmt::Display for CostMatrix {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut i = 0;
        writeln!(f)?;
        for _y in 0..self.size {
            for _x in 0..self.size {
                write!(f, "{:.0}\t", self.data[i].unwrap_or(f64::NAN))?;
                i += 1;
            }
            writeln!(f)?;
        }
        Ok(())
    }
}
impl CostMatrix {
    pub fn new(points: &Vec<Point>) -> Self {
        let size = points.len();
        let mut data = vec![None; size * size];
        let mut i = 0;
        for y in 0..size {
            for x in 0..size {
                if x != y {
                    let dist = points[x].dist_to(&points[y]);
                    data[i] = Some(dist);
                }
                i += 1;
            }
        }
        Self { data, size }
    }
    fn get_idx(&self, x: usize, y: usize) -> usize {
        self.size * y + x
    }
    pub fn get(&self, x: usize, y: usize) -> &Option<f64> {
        &self.data[self.get_idx(x, y)]
    }
}
