mod angle_list;
mod cost_matrix;
mod line;

use angle_list::AngleOkList;
use cost_matrix::CostMatrix;
use imageproc::drawing::draw_text_mut;
use line::Line;
use rusttype::Font;
use rusttype::Scale;
use std::collections::HashMap;
use std::collections::HashSet;

use std::{collections::BinaryHeap, f64::consts::PI, fs, rc::Rc, time::Instant};

use clap::Parser;
use image::{Rgba, RgbaImage};
use imageproc::drawing::{draw_filled_circle_mut, draw_line_segment_mut};
use rand::{prelude::*, rngs::ThreadRng, Rng};

use crate::line::find_lines;

#[derive(Debug)]
struct Dir(f64, f64);
impl Dir {
    fn angle_to(&self, other: &Dir) -> f64 {
        let dot = self.0 * other.0 + self.1 * other.1;
        f64::acos(dot / (self.len() * other.len())) * 180.0 / PI
    }
    fn len(&self) -> f64 {
        f64::sqrt(self.0 * self.0 + self.1 * self.1)
    }
}
fn angle_ok(a: f64) -> bool {
    //cut off up to the 3rd decimal place
    let a = (a * 1000.0).round() / 1000.0;
    a <= 90.0 || a >= 270.0
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point(f64, f64);
impl TryFrom<Vec<&str>> for Point {
    type Error = std::num::ParseFloatError;
    fn try_from(value: Vec<&str>) -> Result<Self, Self::Error> {
        assert!(value.len() == 2, "{:?}", value);
        let width = value[0].parse()?;
        let height = value[1].parse()?;
        Ok(Point(width, height))
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
    fn as_tuple(&self) -> (f32, f32) {
        (self.0 as f32, self.1 as f32)
    }
}
fn get_points(size: (f64, f64), n: usize, rng: &mut ThreadRng) -> Vec<Point> {
    (0..n).map(|_| Point::new_rand(size, rng)).collect()
}
fn get_len(path: &[Point]) -> f64 {
    //println!("get_len({:?})", path);
    let mut result = 0.0;
    let mut p_pt = &path[0];
    for pt in path.iter().skip(1) {
        result += pt.dist_to(p_pt);
        p_pt = pt;
    }
    result
}
fn path_is_ok(path: &[&Point]) -> bool {
    let mut p_pt = &path[0];
    let mut p_dir = p_pt.dir_to(path[1]);
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
        let prev = self.prev.as_ref().map(|prev| prev.pt);
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
        let mut path = vec![pts[node.pt]];
        while let Some(new_node) = &node.prev {
            path.push(pts[new_node.pt]);
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
    fn eq(&self, _other: &Self) -> bool {
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
    fn cmp(&self, _other: &Self) -> std::cmp::Ordering {
        todo!()
    }
}
fn check_angles(pts: &Vec<Point>) {
    let mut p_pt: Option<Point> = None;
    let mut p_dir: Option<Dir> = None;
    for pt in pts {
        if let Some(p_pt) = p_pt {
            let new_dir = p_pt.dir_to(pt);
            if let Some(p_dir) = p_dir {
                let angle = p_dir.angle_to(&new_dir);
                if !angle_ok(angle) {
                    println!("angle: {}", angle);
                    println!("p_dir: {:?}", p_dir);
                    println!("new_dir: {:?}", new_dir);
                    panic!("angle not ok");
                }
            }
            p_dir = Some(new_dir);
        }
        p_pt = Some(*pt);
    }
    println!("all angles ok");
}
#[derive(Clone)]
struct Skip {
    end: usize,
    len: usize,
}
impl Skip {
    fn new(end: usize, len: usize) -> Self {
        Self { end, len }
    }
}
enum NextPaths {
    Multiple(usize, Vec<usize>),
    Skip(Skip),
}
//function that makes a line of n points
fn find_shortest_paths(costs: &CostMatrix, lines: &Vec<Line>) -> Vec<(Vec<usize>, Option<usize>)> {
    let line_pts: HashSet<usize> = lines
        .iter()
        .flat_map(|l| -> Vec<usize> { (l.pts[1..l.pts.len() - 1]).to_vec() })
        .collect();
    let mut line_len = 0;
    for line in lines {
        line_len += line.pts.len();
    }
    println!(
        "line_len: {}, middle_len: {}, diff: {}",
        line_len,
        line_pts.len(),
        line_len - line_pts.len()
    );

    let line_skips: HashMap<usize, Skip> = lines
        .iter()
        .flat_map(|l| {
            [(0, 1), (1, 0)]
                .into_iter()
                .map(|(a, b)| (l.ends[a], Skip::new(l.ends[b], l.pts.len())))
        })
        .collect();
    let n_pts = costs.size;
    (0..n_pts)
        .map(|start| -> (Vec<usize>, Option<usize>) {
            let skip = line_skips.get(&start).map(|skip| skip.end);
            /*if line_pts.contains(&start) {
                return None;
            }*/
            let mut nexts = (0..n_pts)
                .filter_map(|b| {
                    if line_pts.contains(&b) {
                        return None;
                    }
                    Some((b, (*costs.get(start, b))?))
                })
                .collect::<Vec<_>>();
            nexts.sort_by(|a, b| a.1.total_cmp(&b.1));
            let max_len = nexts[1].1 * 2.0;
            //calculate the largest index that is still ok
            let mut max_idx = nexts.len();
            for (i, (_b, cost)) in nexts.iter().enumerate() {
                if cost > &max_len {
                    max_idx = i;
                    break;
                }
            }
            let nexts = nexts.into_iter().map(|(b, _)| b).collect();
            (nexts, skip)
        })
        .collect::<Vec<_>>()
}
//fügt die linien wieder in die liste ein
fn insert_lines(path: Vec<usize>, lines: Vec<Line>) -> Vec<usize> {
    let mut line_map = lines
        .iter()
        .flat_map(|l| {
            [0, 1].into_iter().map(move |i| {
                if i == 0 {
                    (l.ends[i], l.pts.clone())
                } else {
                    (l.ends[i], l.pts.iter().rev().cloned().collect())
                }
            })
        })
        .collect::<HashMap<_, _>>();
    let mut new_path = vec![];
    let mut p_skipped = false;
    for (_i, pt) in path.into_iter().enumerate() {
        if let Some(line) = line_map.remove(&pt) {
            if !p_skipped {
                p_skipped = true;
                new_path.extend(line[..line.len() - 1].iter().cloned());
                continue;
            }
        }
        p_skipped = false;
        new_path.push(pt);
    }
    new_path
}
fn find_n_n(costs: &CostMatrix, angle_list: &AngleOkList) -> Option<Vec<usize>> {
    //println!("costs: {}", costs);
    //println!("angle_list: {:?}, {}", angle_list, angle_list.is_ok(0, 1, 2));
    println!("angle ok: {}", angle_list.is_ok(19, 73, 7));
    let lines = find_lines(costs, angle_list);
    println!(
        "lines: {:?}",
        lines.iter().map(|l| l.ends).collect::<Vec<_>>()
    );
    //findet die nähsten punkte zu jedem punkt
    let short_paths = find_shortest_paths(costs, &lines);

    let start_pt = if lines.is_empty() {
        0
    } else {
        lines[0].ends[0]
    };
    //println!("short_paths: {:?}", short_paths);
    let mut path = vec![start_pt];
    let mut idxs = vec![0];
    let mut skips = vec![false];
    let mut n_pts = costs.size;
    let mut free_pts = vec![true; n_pts];
    //subtract the points that are on lines
    for line in &lines {
        n_pts -= line.pts.len() - 2;
    }
    println!("n_pts: {}", n_pts);
    free_pts[start_pt] = false;
    let mut i: u128 = 0;
    let mut n_back = 0;
    'main_loop: loop {
        i += 1;
        //*
        if i > (n_pts * n_pts * n_pts * n_pts) as u128 {
            //println!("this is taking too long, abort");
            //return None;
        }
        //*/
        // println!("path: {path:?}, idxs: {idxs:?}");
        if idxs.is_empty() {
            panic!("idxs is empty");
            return None;
        }
        let level = idxs.len() - 1;

        let curr_pt = path[level];
        let prev_was_skip = skips[level];

        //println!("level: {}, i: {}", level, i);
        let p_pt = if level > 0 {
            path.get(level - 1).cloned()
        } else {
            None
        };
        let mut curr_idx = idxs.pop().unwrap();
        let (next_pts, skip) = &short_paths[curr_pt];
        let mut did_skip = false;
        let next_pt = if skip.is_some() && !prev_was_skip {
            if curr_idx == 0 && p_pt.is_none()
                || angle_list.is_ok(p_pt.unwrap(), curr_pt, skip.unwrap())
            {
                did_skip = true;
                skip.unwrap()
            } else {
                free_pts[curr_pt] = true;
                path.pop();
                skips.pop();

                continue 'main_loop;
            }
        } else {
            //skips.push(false);
            loop {
                // falls alle nächsten punkte ausgeschlossen wurden
                // wird wieder zurückgegangen
                if curr_idx >= next_pts.len() {
                    n_back += 1;

                    //return Some(path);
                    if i % 10_000_000 == 0 {
                        println!("backtracking, len: {}, path: {:?}", i, path);
                        // return Some(path);
                    }
                    //println!("backtracking, len: {}, max_idx: {}", i, max_idx);
                    free_pts[curr_pt] = true;
                    path.pop();
                    skips.pop();

                    continue 'main_loop;
                }
                let next_pt = next_pts[curr_idx];
                //falls der näheste punkt schon benutzt wurde oder der winkel nicht passt
                if !free_pts[next_pt]
                    || p_pt.is_some() && !angle_list.is_ok(p_pt.unwrap(), curr_pt, next_pt)
                {
                    curr_idx += 1; //wird der nächstbeste punkt genommen
                } else {
                    break next_pt;
                }
            }
        };

        path.push(next_pt);
        skips.push(did_skip);
        if path.len() == n_pts {
            println!("path len: {}, skip len: {}", path.len(), skips.len());
            let full_path = insert_lines(path, lines);
            println!("i: {}, n_back: {}", i, n_back);
            //let full_path = path;
            //println!("path: {:?}", full_path);
            //println!("skip: {:?}", skips);
            return Some(full_path);
        }
        //println!("moving to next point: {}", next_pt);
        free_pts[next_pt] = false;
        idxs.push(curr_idx + 1);
        idxs.push(0);
    }
    // todo!()
}
fn idxs_to_pts(idxs: Vec<usize>, pts: &[Point]) -> Vec<Point> {
    idxs.into_iter().map(|i| pts[i]).collect()
}
fn find_path_straight(
    start_node: Node,
    angle_list: &AngleOkList,
    upper_bound: &mut f64,
    pts: &[Point],
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
            let (_last_pt, curr_nodes) = &mut stack[level];
            curr_nodes.pop().unwrap()
        };
        if node.cost > *upper_bound {
            // panic!("cost too high");
            continue;
        }
        // println!("1");
        let (p_p_pt, p_pt) = node.path.get_last_two();
        if free_pts.len() == 1 {
            let arr = node.path.to_pts_arr(pts.to_vec());
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
        free_pts.retain(|p| p != &p_pt);
    }
}
fn nearest_neighbour(pts: Vec<Point>) -> Option<(Vec<Point>, f64)> {
    let angle_list = AngleOkList::new(&pts);
    let _start_matrix = CostMatrix::new(&pts);
    //let mut path = None;
    //finding a path using the nearest neighbour heuristic
    for i in 0..pts.len() {
        println!("i: {}", i);
        let pts = pts.clone();
        //pts.swap(0, i);
        let costs = CostMatrix::new(&pts);
        if let Some(p) = find_n_n(&costs, &angle_list) {
            println!("found path with length: {}", p.len());
            let path = idxs_to_pts(p, &pts);
            let double = path[1];
            for pt in &pts {
                if pt == &double {
                    println!("double: {:?}", pt);
                }
            }
            return Some((path, 0.0));
            //path = Some(p);
        }
    }
    None
}
fn find_path_jump(pts: Vec<Point>) -> Option<(Vec<Point>, f64)> {
    let angle_list = AngleOkList::new(&pts);
    let mut start_matrix = CostMatrix::new(&pts);
    let path = None;
    //finding a path using the nearest neighbour heuristic
    for i in 0..pts.len() {
        println!("i: {}", i);
        let pts = pts.clone();
        //pts.swap(0, i);
        let costs = CostMatrix::new(&pts);
        if let Some(p) = find_n_n(&costs, &angle_list) {
            println!("found path with length: {}", p.len());
            let path = idxs_to_pts(p, &pts);
            let double = path[1];
            for pt in &pts {
                if pt == &double {
                    println!("double: {:?}", pt);
                }
            }
            return Some((path, 0.0));
            //path = Some(p);

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
    let path = best_path.path.to_pts_arr(pts);
    println!("path: {:?}", path);
    //path.pop();
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
fn calc_bounds(path: &[Point]) -> (f64, f64) {
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
    let max = max_x.max(max_y) + 6.0;
    (min, max)
}
fn draw_path(
    path: &[Point],
    image: &mut RgbaImage,
    pt_bounds: (f64, f64),
    color: [u8; 4],
    pt_color: [u8; 4],
    //offset: f64,
    show_labels: bool,
    show_connections: bool,
) {
    //get the size of the image
    let size = image.dimensions();
    let size = (size.0 as f64, size.1 as f64);
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
    let path = path
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
        draw_filled_circle_mut(image, (pt.0 as i32, pt.1 as i32), 5, Rgba(pt_color));
        let label = i.to_string(); //+ ": " + &angle.to_string() + "°";
                                   //println!("label for {:?}: {}", pt, label);
        if show_labels {
            draw_text_mut(
                image,
                Rgba(pt_color),
                pt.0 as i32,
                pt.1 as i32 - 20,
                scale,
                &font,
                &label,
            );
        }
        p_pt = Some(pt);
    }
}
fn load_pts(path: &str) -> Vec<Point> {
    let s = fs::read_to_string(path).unwrap();
    s.split('\n')
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
    let _rng = rand::thread_rng();
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

    //let mut points = get_points((size, size), 7, &mut rng);
    //let points = make_line(40, Point(0.0,0.0), Dir(1.0, 0.0));
    //println!("points len: {}", points.len());
    let points = load_pts("data/wenigerkrumm2.txt");

    let pt_bounds = calc_bounds(&points);
    //points.shuffle(&mut rng);
    let _pts_len = points.len();
    // points.swap(0, rng.gen_range(0..pts_len));
    // let points = vec![
    //     Point(100.0, 200.0),
    //     Point(500.0, 200.0),
    //     Point(600.0, 300.0),
    // ];
    // println!("Points: {:#?}", points);
    println!("new points list");

    let _do_bruteforce = false;
    let mut image = RgbaImage::from_fn(size as u32, size as u32, |_, _| {
        Rgba([0u8, 0u8, 0u8, 255u8])
    });
    if true {
        draw_path(
            &points,
            &mut image,
            //size:
            pt_bounds,
            [0u8, 0u8, 255u8, 255u8],
            [0u8, 0u8, 255u8, 255u8],
            //size - 700.0,
            false,
            false,
        );
    }
    if true {
        let costs = CostMatrix::new(&points);
        let angle_list = AngleOkList::new(&points);
        let lines = find_lines(&costs, &angle_list);
        for line in lines {
            println!("line: {:?}", line);
            let line = idxs_to_pts(line.pts, &points);
            draw_path(
                &line,
                &mut image,
                pt_bounds,
                [255u8, 0u8, 0u8, 255u8],
                [255u8, 0u8, 0u8, 255u8],
                //size - 700.0,
                true,
                true,
            );
        }
        //image.save("out.png").unwrap();
        //return;
    }

    if true {
        let min_path = nearest_neighbour(points.clone());
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
                pt_bounds,
                [0u8, 0u8, 255u8, 255u8],
                [0u8, 255u8, 0u8, 255u8],
                //size - 700.0,
                true,
                true,
            );

            //break;
        } else {
            println!("didn't find any possible paths");
        }
    }
    image.save("out.png").unwrap();
    println!("saved image");
}
