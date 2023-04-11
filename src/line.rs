use std::collections::{HashMap, LinkedList};

use crate::{angle_list::AngleOkList, CostMatrix, Dir, Point};

#[derive(Debug, Clone)]
pub struct Line {
    pub ends: [usize; 2],
    pub pts: Vec<usize>,
}
impl Line {
    fn new(pts: Vec<usize>) -> Self {
        let ends = [pts[0], pts[pts.len() - 1]];
        Self { ends, pts }
    }
    pub fn get_cost(&self, costs: &CostMatrix) -> f64 {
        let mut cost = 0.0;
        for i in 0..self.pts.len() - 1 {
            cost += costs.get(self.pts[i], self.pts[i + 1]).unwrap();
        }
        cost
    }
}
fn _make_line(n: usize, start: Point, dir: Dir) -> Vec<Point> {
    let mut pts = vec![start];
    let mut last_pt = start;
    for _ in 1..n {
        let new_pt = Point(last_pt.0 + dir.0, last_pt.1 + dir.1);
        pts.push(new_pt);
        last_pt = new_pt;
    }
    pts
}
pub fn find_lines(costs: &CostMatrix, angle_list: &AngleOkList) -> Vec<Line> {
    //find lines in the points by finding points that only have two points that are really close
    let n_pts = costs.size;
    let mut line_segments = HashMap::new();
    //finding line segments
    for start in 0..n_pts {
        let mut nexts = (0..n_pts)
            .filter_map(|b| Some((b, (*costs.get(start, b))?)))
            .collect::<Vec<_>>();
        nexts.sort_by(|a, b| a.1.total_cmp(&b.1));
        let (pt_0, dist_0) = nexts[0];
        let (pt_1, dist_1) = nexts[1];
        let (_, dist_2) = nexts[2];
        let security = 1.5;
        if dist_1 < dist_0 * security
            && dist_2 > dist_1 * 1.5
            && angle_list.is_ok(pt_0, start, pt_1)
        {
            line_segments.insert(start, (pt_0, pt_1));
        }
    }

    let mut lines = vec![];
    let mut used_pts = vec![false; n_pts];
    while !line_segments.is_empty() {
        let mut line = LinkedList::new();
        //get random key
        let pt = *line_segments.keys().next().unwrap();
        let (mut start, mut end) = line_segments.remove(&pt).unwrap();
        line.push_back(start);
        line.push_back(pt);
        line.push_back(end);
        if used_pts[pt] || used_pts[start] || used_pts[end] {
            continue;
        }

        used_pts[pt] = true;
        used_pts[start] = true;
        used_pts[end] = true;
        let mut p_pt = pt;
        while let Some((a, b)) = line_segments.remove(&end) {
            let new_end = if a == p_pt { b } else { a };
            if used_pts[new_end] {
                break;
            }
            line.push_back(new_end);
            used_pts[new_end] = true;
            p_pt = end;
            end = new_end;
        }
        p_pt = pt;
        while let Some((a, b)) = line_segments.remove(&start) {
            let new_start = if a == p_pt { b } else { a };
            if used_pts[new_start] {
                break;
            }
            line.push_front(new_start);
            used_pts[new_start] = true;
            p_pt = start;
            start = new_start;
        }
        //transform line into a vector
        let line = line.into_iter().collect::<Vec<_>>();
        if line.len() > 4 {
            lines.push(Line::new(line));
        }
    }
    lines
}
