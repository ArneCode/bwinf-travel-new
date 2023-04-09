mod angle_list;

use angle_list::AngleOkList;
use imageproc::drawing::draw_text_mut;
use rusttype::Font;
use rusttype::Scale;
use std::collections::HashMap;
use std::collections::LinkedList;
use std::{collections::BinaryHeap, f64::consts::PI, fmt, fs, mem, rc::Rc, time::Instant};

use clap::Parser;
use image::{Rgba, RgbaImage};
use imageproc::drawing::{draw_filled_circle_mut, draw_line_segment_mut};
use rand::{prelude::*, rngs::ThreadRng, seq::SliceRandom, Rng};

#[derive(Debug)]
struct Dir(f64, f64);
impl Dir {
    fn angle_to(&self, other: &Dir) -> f64 {
        let dot = self.0 * other.0 + self.1 * other.1;
        f64::acos(dot / (self.len() * other.len()))
    }
    fn len(&self) -> f64 {
        f64::sqrt(self.0 * self.0 + self.1 * self.1)
    }
}
fn angle_ok(a: f64) -> bool {
    //cut off up to the 3rd decimal place
    let a = (a * 1000.0).round() / 1000.0;
    a <= PI / 2.0 || a >= 3.0 * PI / 2.0
}
#[derive(Debug, Clone, Copy)]
pub struct Point(f64, f64);
impl TryFrom<Vec<&str>> for Point {
    type Error = std::num::ParseFloatError;
    fn try_from(value: Vec<&str>) -> Result<Self, Self::Error> {
        assert!(value.len() == 2, "{:?}", value);
        let width = value[0].parse()?;
        let height = value[1].parse()?;
        if width > height {
            Ok(Self(width, height))
        } else {
            Ok(Self(height, width))
        }
    }
}

impl Point {
    fn dir_to(&self, other: &Point) -> Dir {
        Dir(other.0 - self.0, other.1 - self.1)
    }
    fn new_rand(range: (f64, f64), rng: &mut rand::prelude::ThreadRng) -> Self {
        Point(rng.gen_range(0.0..range.0), rng.gen_range(0.0..range.1))
    }
    fn dist_to(&self, other: &Point) -> f64 {
        let x = self.0 - other.0;
        let y = self.1 - other.1;
        f64::sqrt(x * x + y * y)
    }
    fn to_tuple(&self) -> (f32, f32) {
        (self.0 as f32, self.1 as f32)
    }
}
fn get_points(size: (f64, f64), n: usize, rng: &mut ThreadRng) -> Vec<Point> {
    (0..n).map(|_| Point::new_rand(size, rng)).collect()
}
fn get_len(path: &Vec<Point>) -> f64 {
    //println!("get_len({:?})", path);
    let mut result = 0.0;
    let mut p_pt = &path[0];
    for pt in path.iter().skip(1) {
        result += pt.dist_to(p_pt);
        p_pt = pt;
    }
    result
}
fn path_is_ok(path: &Vec<&Point>) -> bool {
    let mut p_pt = &path[0];
    let mut p_dir = p_pt.dir_to(&path[1]);
    for pt in path.iter().skip(1) {
        let new_dir = p_pt.dir_to(pt);
        p_pt = pt;
        let angle = p_dir.angle_to(&new_dir);
        if !angle_ok(angle) {
            return false;
        }
        p_dir = new_dir;
    }
    true
}

#[derive(Clone, Debug)]
struct CostMatrix {
    data: Vec<Option<f64>>,
    size: usize,
}
impl fmt::Display for CostMatrix {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut i = 0;
        writeln!(f, "")?;
        for y in 0..self.size {
            for x in 0..self.size {
                write!(f, "{:.0}\t", self.data[i].unwrap_or(f64::NAN))?;
                i += 1;
            }
            writeln!(f, "")?;
        }
        Ok(())
    }
}
impl CostMatrix {
    fn new(points: &Vec<Point>) -> Self {
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
    fn get(&self, x: usize, y: usize) -> &Option<f64> {
        &self.data[self.get_idx(x, y)]
    }
    fn set(&mut self, x: usize, y: usize, value: Option<f64>) {
        let i = self.get_idx(x, y);
        self.data[i] = value;
    }
    fn clear_line(&mut self, n: usize, is_vertical: bool) {
        let size = self.size;
        let start = if is_vertical { n * size } else { n };
        let step = if is_vertical { 1 } else { size };
        for x in 0..size {
            self.data[start + x * step] = None;
        }
    }
    ///
    ///```
    ///assert!(false);
    ///```
    fn reduce_line(&mut self, n: usize, is_vertical: bool) -> f64 {
        let size = self.size;
        let start = if is_vertical { n * size } else { n };
        let step = if is_vertical { 1 } else { size };
        let min = (0..size)
            .filter_map(|x| self.data[start + x * step])
            .fold(f64::NAN, f64::min);
        if min.is_nan() {
            return 0.0;
        }
        for x in 0..size {
            if let Some(v) = &mut self.data[start + x * step] {
                *v -= min;
            }
        }
        min
    }
    fn reduce_lines(&mut self, is_vertical: bool) -> f64 {
        let mut sum = 0.0;
        for i in 0..self.size {
            sum += self.reduce_line(i, is_vertical);
        }
        sum
    }
    fn reduce(&mut self) -> f64 {
        // println!("reducing matrix, before: {}", self);
        let cost = self.reduce_lines(false) + self.reduce_lines(true);
        // println!("after: {self}, cost: {cost}");
        cost
    }
    fn add_path(&self, start: usize, end: usize) -> Option<(f64, CostMatrix)> {
        // println!("adding path, start: {start}, end: {end}");
        let mut matrix = self.clone();
        //cost: cost from start to end + prev_cost + new reduction cost
        let i = self.get_idx(end, start);
        // let i = self.get_idx(start, end);
        // let move_cost = mem::take(&mut matrix.data[i]).expect("path wasn't available");
        let move_cost = mem::take(&mut matrix.data[i])?;
        //rows for start
        matrix.clear_line(start, true);
        //columns for end
        matrix.clear_line(end, false);
        //remove the other way round
        matrix.set(start, end, None);
        // let reduce_cost = matrix.reduce();
        // let cost = move_cost + reduce_cost;
        Some((move_cost, matrix))
    }
}
#[derive(Clone)]
struct PathPt {
    prev: Option<Rc<PathPt>>,
    pt: usize,
}
impl PathPt {
    fn new(pt: usize) -> Self {
        PathPt { prev: None, pt }
    }
    fn add(self: &Rc<Self>, pt: usize) -> Self {
        Self {
            prev: Some(self.clone()),
            pt,
        }
    }
    fn get_last_two(&self) -> (Option<usize>, usize) {
        let prev = if let Some(prev) = &self.prev {
            Some(prev.pt)
        } else {
            None
        };
        (prev, self.pt)
    }
    fn to_arr(&self) -> Vec<usize> {
        let mut node = Rc::new(self.clone());
        let mut path = vec![node.pt];
        while let Some(new_node) = &node.prev {
            path.push(new_node.pt);
            node = new_node.clone();
        }
        path.into_iter().rev().collect()
    }
    fn to_pts_arr(&self, pts: Vec<Point>) -> Vec<Point> {
        let mut node = Rc::new(self.clone());
        let mut path = vec![pts[node.pt].clone()];
        while let Some(new_node) = &node.prev {
            path.push(pts[new_node.pt].clone());
            node = new_node.clone();
        }
        path.into_iter().rev().collect()
    }
}
#[derive(Clone)]
struct Node {
    costs: CostMatrix,
    cost: f64,
    path: Rc<PathPt>,
    level: usize,
}

impl Node {
    fn new(costs: CostMatrix, cost: f64, path: Rc<PathPt>, level: usize) -> Self {
        Self {
            costs,
            cost,
            path,
            level,
        }
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        todo!()
    }
}
impl Eq for Node {}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        other.cost.partial_cmp(&self.cost)
    }
}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        todo!()
    }
}
fn check_angles(pts: &Vec<Point>) {
    let mut p_pt = pts.last().unwrap();
    let mut p_p_pt = &pts[pts.len() - 2];
    for pt in pts {
        let dir1 = p_p_pt.dir_to(&p_pt);
        let dir2 = p_pt.dir_to(pt);
        let angle = dir1.angle_to(&dir2);
        if !angle_ok(angle) {
            //println!("angle not ok");
        } else {
            //println!("angle ok: {:?} -> {:?} -> {:?}", p_p_pt, p_pt, pt);
        }
        p_p_pt = p_pt;
        p_pt = pt;
    }
    println!("all angles ok!")
}
fn find_lines(costs: &CostMatrix, angle_list: &AngleOkList) -> Vec<Vec<usize>> {
    //find lines in the points by finding points that only have two points that are really close
    let n_pts = costs.size;
    let mut line_segments = HashMap::new();
    //finding line segments
    for start in 0..n_pts {
        let mut nexts = (0..n_pts)
            .filter_map(|b| Some((b, costs.get(start, b).clone()?)))
            .collect::<Vec<_>>();
        nexts.sort_by(|a, b| a.1.total_cmp(&b.1));
        let (pt_0, dist_0) = nexts[0];
        let (pt_1, dist_1) = nexts[1];
        let (_, dist_2) = nexts[2];
        let security = 1.5;
        if dist_1 < dist_0 * security && dist_2 > dist_0 * security && angle_list.is_ok(pt_0, start, pt_1) {
            line_segments.insert(start, (pt_0, pt_1));
        }
    }
    let mut lines = vec![];
    while !line_segments.is_empty() {
        let mut line = LinkedList::new();
        //get random key
        let pt = *line_segments.keys().next().unwrap();
        let (mut start, mut end) = line_segments.remove(&pt).unwrap();
        line.push_back(start);
        line.push_back(pt);
        line.push_back(end);
        let mut p_pt = pt;
        while let Some((new_start, new_end)) = line_segments.remove(&end) {
            let new_end = if new_start == p_pt {
                new_end
            } else {
                new_start
            };
            line.push_back(new_end);
            p_pt = end;
            end = new_end;
        }
        while let Some((new_start, new_end)) = line_segments.remove(&start) {
            let new_start = if new_start == p_pt {
                new_end
            } else {
                new_start
            };
            line.push_front(new_start);
            p_pt = start;
            start = new_start;
        }
        //transform line into a vector
        let line = line.into_iter().collect::<Vec<_>>();
        if line.len() > 5 ||true {
            lines.push(line);
        }
    }
    lines
}
//function that makes a line of n points
fn find_shortest_paths(costs: &CostMatrix) -> Vec<(usize, Vec<usize>)> {
    let n_pts = costs.size;
    (0..n_pts)
        .map(|start| -> (usize, Vec<usize>) {
            let mut nexts = (0..n_pts)
                .filter_map(|b| Some((b, costs.get(start, b).clone()?)))
                .collect::<Vec<_>>();
            nexts.sort_by(|a, b| a.1.total_cmp(&b.1));
            let max_len = nexts[1].1 * 2.0;
            //calculate the largest index that is still ok
            let mut max_idx = nexts.len();
            for (i, (b, cost)) in nexts.iter().enumerate() {
                if cost > &max_len {
                    max_idx = i;
                    break;
                }
            }
            let nexts = nexts.into_iter().map(|(b, _)| b).collect();
            (max_idx, nexts)
        })
        .collect::<Vec<_>>()
}
fn find_n_n(costs: &CostMatrix, angle_list: &AngleOkList) -> Option<Vec<usize>> {
    //println!("costs: {}", costs);
    //println!("angle_list: {:?}, {}", angle_list, angle_list.is_ok(0, 1, 2));
    let n_pts = costs.size;
    //findet die nähsten punkte zu jedem punkt
    let short_paths: Vec<(usize, Vec<usize>)> = find_shortest_paths(costs);
    //println!("short_paths: {:?}", short_paths);
    let mut path = vec![0];
    let mut idxs = vec![0];
    let mut free_pts = vec![true; n_pts];
    free_pts[0] = false;
    let mut i: u128 = 0;
    let mut n_back = 0;
    'main_loop: loop {
        i += 1;
        //*
        if i > (n_pts * n_pts * n_pts * n_pts) as u128 {
            println!("this is taking too long, abort");
            return None;
        }
        //*/
        // println!("path: {path:?}, idxs: {idxs:?}");
        if idxs.is_empty() {
            panic!("idxs is empty");
            return None;
        }
        let level = idxs.len() - 1;

        let curr_pt = path[level];

        //println!("level: {}, i: {}", level, i);
        let p_pt = if level > 0 {
            path.get(level - 1).cloned()
        } else {
            None
        };
        let mut curr_idx = idxs.pop().unwrap();
        let next_pt = loop {
            let (max_idx, next_pts) = &short_paths[curr_pt];
            // falls alle nächsten punkte ausgeschlossen wurden
            // wird wieder zurückgegangen
            if curr_idx + 1 >= n_pts {
                n_back += 1;
                if n_back > 1000000 {
                    //return Some(path);
                }
                //return Some(path);
                if i % 1_000_000 == 0 {
                    println!("backtracking, len: {}, max_idx: {}", i, max_idx);
                }
                //println!("backtracking, len: {}, max_idx: {}", i, max_idx);
                free_pts[curr_pt] = true;
                path.pop();

                continue 'main_loop;
            }
            let mut next_pt = next_pts[curr_idx];
            //falls der näheste punkt schon benutzt wurde oder der winkel nicht passt
            if !free_pts[next_pt]
                || p_pt.is_some() && !angle_list.is_ok(p_pt.unwrap(), curr_pt, next_pt)
            {
                curr_idx += 1; //wird der nächstbeste punkt genommen
            } else {
                break next_pt;
            }
        };
        /*
        //falls der näheste punkt schon benutzt wurde oder der winkel nicht passt
        while !free_pts[next_pt]
            || p_pt.is_some() && !angle_list.is_ok(p_pt.unwrap(), curr_pt, next_pt)
        {
            //println!("taking next point instead current: {}, next_pt: {}", curr_pt, next_pt);
            curr_idx += 1;//wird der nächstbeste punkt genommen
            // falls alle nächsten punkte ausgeschlossen wurden
            // wird wieder zurückgegangen
            if curr_idx + 1 >= n_pts {
                print!("backtracking, len: {}", idxs.len());
                free_pts[curr_pt] = true;
                path.pop();
                // println!("continuing, len: {}", idxs.len());
                continue 'main_loop;
            }
            // println!("curr_pt: {curr_pt}, idx: {curr_idx}");
            next_pt = short_paths[curr_pt][curr_idx];
        }
        */
        path.push(next_pt);
        if path.len() == n_pts {
            return Some(path);
        }
        //println!("moving to next point: {}", next_pt);
        free_pts[next_pt] = false;
        idxs.push(curr_idx + 1);
        idxs.push(0);
    }
    // todo!()
}
fn idxs_to_pts(idxs: Vec<usize>, pts: &Vec<Point>) -> Vec<Point> {
    idxs.into_iter().map(|i| pts[i]).collect()
}
fn find_path_straight(
    start_node: Node,
    angle_list: &AngleOkList,
    upper_bound: &mut f64,
    pts: &Vec<Point>,
) -> Option<Node> {
    let size = start_node.costs.size;
    //maybe wrong get call
    let mut free_pts = (0..size)
        .filter(|dest| start_node.costs.get(start_node.path.pt, *dest).is_some())
        .collect::<Vec<_>>();
    let mut stack = vec![(0, vec![start_node])];
    let mut best_solution = None;
    let mut n_searched = 0;
    loop {
        n_searched += 1;
        let level = stack.len() - 1;
        // println!("level: {level}, free: {:?}", free_pts);
        if stack[level].1.is_empty() {
            if level == 0 {
                println!("found best after: {}", n_searched);
                return best_solution;
            }
            // println!("pushing {} to free", stack[level].0);
            free_pts.push(stack[level].0);
            stack.pop();
            continue;
        }
        let node = {
            let (last_pt, curr_nodes) = &mut stack[level];
            let curr_node = curr_nodes.pop().unwrap();
            curr_node
        };
        if node.cost > *upper_bound {
            // panic!("cost too high");
            continue;
        }
        // println!("1");
        let (p_p_pt, p_pt) = node.path.get_last_two();
        if free_pts.len() == 1 {
            let arr = node.path.to_pts_arr(pts.clone());
            let a = &arr[arr.len() - 2];
            let b = &arr[0];
            let c = &arr[1];
            let dir1 = a.dir_to(b);
            let dir2 = b.dir_to(c);
            let angle = dir1.angle_to(&dir2);
            if !angle_ok(angle) {
                // panic!("not ok: {}, {dir1:?}, {dir2:?}", angle);
                continue;
            } //done
              // panic!("done");
            *upper_bound = node.cost;
            best_solution = Some(node);
            println!("new best");
            return best_solution;
        }
        // println!("2");
        let mut new_nodes = free_pts
            .iter()
            .filter_map(|next| {
                if next == &p_pt {
                    return None;
                }
                if let Some(p_p_pt) = p_p_pt {
                    if !angle_list.is_ok(p_p_pt, p_pt, *next) {
                        return None;
                    }
                }
                let (move_cost, mut new_costs) = node.costs.add_path(p_pt, *next)?;
                let cost = node.cost + move_cost + new_costs.reduce();
                if cost > *upper_bound {
                    return None;
                }
                let node = Node::new(new_costs, cost, node.path.add(*next).into(), node.level + 1);
                Some(node)
            })
            .collect::<Vec<_>>();
        //sort smallest cost last
        new_nodes.sort_by(|a, b| b.cost.total_cmp(&a.cost));
        stack.push((p_pt, new_nodes));
        free_pts = free_pts.into_iter().filter(|p| p != &p_pt).collect();
    }
}

fn find_path_jump(pts: Vec<Point>) -> Option<(Vec<Point>, f64)> {
    let angle_list = AngleOkList::new(&pts);
    let mut start_matrix = CostMatrix::new(&pts);
    let mut path = None;
    //finding a path using the nearest neighbour heuristic
    for i in 0..pts.len() {
        println!("i: {}", i);
        let mut pts = pts.clone();
        pts.swap(0, i);
        let costs = CostMatrix::new(&pts);
        if let Some(p) = find_n_n(&costs, &angle_list) {
            println!("found path with length: {}", p.len());
            path = Some(p);
            println!("found path: {:?}", path);
            break;
        }
    }
    let path = path?;
    let start_cost = start_matrix.reduce();
    let (nodes, mut best_path, mut upper_bound) =
        path_to_nodes(path, start_matrix, start_cost, &angle_list);
    println!("nodes: {}", nodes.len());
    let mut queue = nodes
        .into_iter()
        .filter(|n| n.cost <= upper_bound)
        .map(Rc::new)
        .collect::<BinaryHeap<_>>();
    println!("queue: {}", queue.len());
    //return Some((result?.into_iter().map(|i| pts[i].clone()).collect(), 0.0));
    //let start_node = Node::new(start_matrix, start_cost, Rc::new(PathPt::new(0)), 1);
    //let mut upper_bound = f64::MAX;
    // let result = find_path_straight(start_node, &angle_list, &mut upper_bound, &pts);
    //queue.push(Rc::new(start_node));
    //let mut best_path: Option<Rc<Node>> = None;
    let mut max_len = 0;
    while let Some(node) = queue.pop() {
        break;
        if node.level > max_len {
            max_len = node.level;
        }
        if queue.len() > 30_000 {
            //find average cost of nodes in queue
            let avg_cost = queue.iter().map(|n| n.cost).sum::<f64>() / queue.len() as f64;
            //remove all nodes with a cost higher than the average
            upper_bound = avg_cost;
            queue = queue.into_iter().filter(|n| n.cost <= avg_cost).collect();
            println!("stopped because the queue was getting to long");
            println!("max_len: {}", max_len);
            //break;
        }
        if node.cost > upper_bound {
            println!("cost too high");
            break;
        }
        //if the path is complete:
        if node.level == pts.len() {
            println!("found complete path");
            //check wether angle at path end is ok:
            /*
            let arr = node.path.to_pts_arr(pts.clone());
            let a = &arr[arr.len() - 2];
            let b = &arr[0];
            let c = &arr[1];
            let dir1 = a.dir_to(b);
            let dir2 = b.dir_to(c);
            let angle = dir1.angle_to(&dir2);
            if !angle_ok(angle) {
                // panic!("not ok: {}, {dir1:?}, {dir2:?}", angle);
                continue;
            }
            */
            upper_bound = node.cost;
            best_path = node;
            println!("found new best");
            break;
        }
        let (p_p_pt, p_pt) = node.path.get_last_two();
        if node.level == 1 {
            // panic!("p_pt: {:?}", p_pt);
        }
        queue.extend((0..node.costs.size).filter_map(|next_pt| {
            if let Some(p_p_pt) = p_p_pt {
                if !angle_list.is_ok(p_p_pt, p_pt, next_pt) {
                    // println!("angle not ok");
                    return None;
                } else if node.level == 2 {
                    // let a = &pts[p_p_pt];
                    // let b = &pts[p_pt];
                    // let c = &pts[next_pt];
                    // let angle = a.dir_to(&b).angle_to(&b.dir_to(&c));
                    // println!("passing: {:?}->{:?}->{:?}, angle: {}", a, b, c, angle);
                }
            } else {
                // println!("not checking after level: {}", node.level);
            }
            let (move_cost, mut new_costs) = node.costs.add_path(p_pt, next_pt)?;
            let cost = node.cost + move_cost + new_costs.reduce();
            if cost > upper_bound {
                return None;
            }
            let node = Rc::new(Node::new(
                new_costs,
                cost,
                node.path.add(next_pt).into(),
                node.level + 1,
            ));
            Some(node)
        }));
    }
    let mut path = best_path.path.to_pts_arr(pts);
    println!("path: {:?}", path);
    ///path.pop();
    //println!("max queue len: {}", max_len);
    Some((path, upper_bound))
}
//reuse the path from find_n_n in find_path_jump
fn path_to_nodes(
    mut path: Vec<usize>,
    start_costs: CostMatrix,
    start_cost: f64,
    angle_list: &AngleOkList,
) -> (Vec<Node>, Rc<Node>, f64) {
    let mut nodes = vec![];
    let mut costs = start_costs;
    let mut p_p_pt = None;
    let mut p_pt = path.remove(0);
    let mut p_cost = start_cost;
    //best path is the path, stored as a linked list in form of a pathpt
    let mut best_path = Rc::new(PathPt::new(p_pt));

    for next_path_pt in path {
        nodes.extend((0..costs.size).filter_map(|next_pt| {
            //not going down the path that is already been used
            if next_pt == next_path_pt {
                return None;
            }
            if let Some(p_p_pt) = p_p_pt {
                if !angle_list.is_ok(p_p_pt, p_pt, next_pt) {
                    // println!("angle not ok");
                    return None;
                }
            }
            let (move_cost, mut new_costs) = costs.add_path(p_pt, next_pt)?;
            let cost = p_cost + move_cost + new_costs.reduce();
            let node = Node::new(new_costs, cost, Rc::new(PathPt::new(next_pt)), 1);
            Some(node)
        }));
        p_p_pt = Some(p_pt);
        //calculate the cost of the next path
        let (move_cost, new_costs) = costs.add_path(p_pt, next_path_pt).unwrap();
        costs = new_costs;
        p_cost = p_cost + move_cost + costs.reduce();
        p_pt = next_path_pt;
        best_path = Rc::new(best_path.add(next_path_pt));
    }
    let best_path = Rc::new(Node::new(costs, p_cost, best_path, 1));
    (nodes, best_path, p_cost)
}
fn map_range(from_range: (f64, f64), to_range: (f64, f64), s: f64) -> f64 {
    to_range.0 + (s - from_range.0) * (to_range.1 - to_range.0) / (from_range.1 - from_range.0)
}
fn calc_bounds(path: &Vec<Point>) -> (f64, f64) {
    //find the bounding box of the path
    //min and max x and y
    let min_x = path
        .iter()
        .map(|p| p.0)
        .min_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();

    let max_x = path
        .iter()
        .map(|p| p.0)
        .max_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();
    let min_y = path
        .iter()
        .map(|p| p.1)
        .min_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();
    let max_y = path
        .iter()
        .map(|p| p.1)
        .max_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();
    let min = min_x.min(min_y) - 3.0;
    let max = max_x.max(max_y) + 3.0;
    (min, max)
}
fn draw_path(
    path: &Vec<Point>,
    image: &mut RgbaImage,
    size: (f64, f64),
    pt_bounds: (f64, f64),
    color: [u8; 4],
    pt_color: [u8; 4],
    //offset: f64,
    show_labels: bool,
    show_connections: bool,
) {
    let (min, max) = pt_bounds;
    let height: f32 = 30.0; // to get 80 chars across (fits most terminals); adjust as desired

    // 2x scale in x direction to counter the aspect ratio of monospace characters.
    let scale = Scale {
        x: height * 2.0,
        y: height,
    };
    let font_data = include_bytes!("../font/font.ttf");
    let font =
        Font::try_from_bytes(font_data as &[u8]).expect("error constructing a Font from bytes");

    //map the point coordinates to the image coordinates
    let mut path = path
        .iter()
        .map(|p| {
            Point(
                map_range((min, max), (0.0, size.0), p.0),
                map_range((min, max), (0.0, size.1), p.1),
            )
        })
        .collect::<Vec<Point>>();

    let mut p_pt: Option<&Point> = None;
    for (i, pt) in path.iter().enumerate() {
        if let Some(p_pt) = p_pt {
            if show_connections {
                //draw the line
                draw_line_segment_mut(
                    image,
                    ((p_pt.0) as f32, p_pt.1 as f32),
                    ((pt.0) as f32, pt.1 as f32),
                    Rgba(color),
                );
            }
        }
        //draw the point
        draw_filled_circle_mut(
            image,
            (pt.0 as i32, pt.1 as i32),
            5,
            Rgba(pt_color),
        );
        let label = i.to_string(); //+ ": " + &angle.to_string() + "°";
        //println!("label for {:?}: {}", pt, label);
        if show_labels {
            draw_text_mut(
                image,
                Rgba(pt_color),
                pt.0 as i32 as i32,
                pt.1 as i32 - 20,
                scale,
                &font,
                &label,
            );
        }
        p_pt = Some(pt);
    }
}
fn make_line(n: usize, start: Point, dir: Dir) -> Vec<Point> {
    let mut pts = vec![start];
    let mut last_pt = start;
    for _ in 1..n {
        let new_pt = Point(last_pt.0 + dir.0, last_pt.1 + dir.1);
        pts.push(new_pt.clone());
        last_pt = new_pt;
    }
    pts
}
fn load_pts(path: &str) -> Vec<Point> {
    let s = fs::read_to_string(path).unwrap();
    s.split("\n")
        .filter_map(|line| -> Option<Point> {
            if line.is_empty() {
                None
            } else {
                Some(
                    line.split(' ')
                        .collect::<Vec<&str>>()
                        .try_into()
                        .expect("couldn't parse line"),
                )
            }
        })
        .collect()
}
//arguments
#[derive(Parser)]
struct Args {
    #[clap(short, long)]
    bruteforce: bool,
    #[clap(short, long)]
    branch_and_bound: bool,
    #[clap(short, long)]
    path: String,
}

fn main() {
    let start = Instant::now();
    let size = 4000.0;
    let mut rng = rand::thread_rng();
    // let points = load_pts("data/wenigerkrumm4.txt");
    // if do_bruteforce {
    //     println!("path through permutation: ");
    //     let min_path_unchecked = get_shortest_path(&points, false);

    //     println!("old path len: {}", get_len(&min_path_unchecked.1));

    //     draw_path(
    //         &min_path_unchecked.1,
    //         &mut image,
    //         [255u8, 0u8, 0u8, 255u8],
    //         0.0,
    //     );
    // }
    // let min_path_checked = get_shortest_path(&points, true);

    println!("path through branch and bound: ");
    loop {
        //let mut points = get_points((size, size), 7, &mut rng);
        //let points = make_line(40, Point(0.0,0.0), Dir(1.0, 0.0));
        //println!("points len: {}", points.len());
        let mut points = load_pts("data/wenigerkrumm1.txt");
        let pt_bounds = calc_bounds(&points);
        //points.shuffle(&mut rng);
        let pts_len = points.len();
        // points.swap(0, rng.gen_range(0..pts_len));
        // let points = vec![
        //     Point(100.0, 200.0),
        //     Point(500.0, 200.0),
        //     Point(600.0, 300.0),
        // ];
        // println!("Points: {:#?}", points);
        println!("new points list");

        let do_bruteforce = false;
        let mut image = RgbaImage::from_fn(size as u32, size as u32, |_, _| {
            Rgba([0u8, 0u8, 0u8, 255u8])
        });

        //let min_path = find_path_jump(points);
        let min_path = Some((points.clone(), 0.0));
        if let Some(min_path_bnb) = /*find_path_jump(points)*/ min_path {
            println!(
                "bnb len: {}, cost: {}",
                get_len(&min_path_bnb.0),
                min_path_bnb.1
            );
            check_angles(&min_path_bnb.0);
            // println!("min_path: {:?}", min_path);

            let elapsed = start.elapsed();
            println!("took: {:?}", elapsed);
            draw_path(
                &min_path_bnb.0,
                &mut image,
                //size:
                (size, size),
                pt_bounds,
                [0u8, 0u8, 255u8, 255u8],
                [0u8, 255u8, 0u8, 255u8],
                //size - 700.0,
                false,
                false
            );
            //image.save("out.png").unwrap();
            println!("saved image");
            //break;
        } else {
            println!("didn't find any possible paths");
        }
        if true {
            let costs = CostMatrix::new(&points);
            let angle_list = AngleOkList::new(&points);
            let lines = find_lines(&costs, &angle_list);
            for line in lines {
                println!("line: {:?}", line);
                let line = idxs_to_pts(line, &points);
                draw_path(
                    &line,
                    &mut image,
                    //size:
                    (size, size),
                    pt_bounds,
                    [0u8, 0u8, 255u8, 255u8],
                    [255u8, 0u8, 0u8, 255u8],
                    //size - 700.0,
                    true,
                    true
                );
            }
            image.save("out.png").unwrap();
            //return;
        }
        break;
    }
}
