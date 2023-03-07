use std::{collections::BinaryHeap, f64::consts::PI, fmt, fs, mem, rc::Rc, time::Instant};

use image::{Rgba, RgbaImage};
use imageproc::drawing::draw_line_segment_mut;
use rand::{rngs::ThreadRng, thread_rng, Rng};

struct Dir(f64, f64);
impl Dir {
    fn angle_to(&self, other: &Dir) -> f64 {
        f64::acos((self.0 * other.0 + self.1 * other.1) % 1.)
    }
}
fn angle_ok(a: &Dir, b: &Dir) -> bool {
    let a = a.angle_to(b);
    a <= PI / 2.0 || a >= 3.0 * PI / 2.0
}
#[derive(Debug, Clone)]
struct Point(f64, f64);
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
        if !angle_ok(&p_dir, &new_dir) {
            return false;
        }
        p_dir = new_dir;
    }
    true
}
struct AngleOkList {
    data: Vec<bool>,
    n_points: usize,
}
impl AngleOkList {
    fn get_idx(n_points: usize, idx: (usize, usize, usize)) -> usize {
        n_points * n_points * idx.2 + n_points * idx.1 + idx.0
    }
    fn new(points: &Vec<Point>) -> Self {
        let n_points = points.len();
        let mut data = vec![false; n_points * n_points * n_points];
        for (i3, p3) in points.iter().enumerate() {
            for (i2, p2) in points.iter().enumerate() {
                for (i1, p1) in points.iter().enumerate() {
                    let dir1 = p1.dir_to(p2);
                    let dir2 = p2.dir_to(p3);
                    let idx = Self::get_idx(n_points, (i1, i2, i3));
                    data[idx] = angle_ok(&dir1, &dir2);
                }
            }
        }
        Self { n_points, data }
    }
    fn is_ok(&self, i1: usize, i2: usize, i3: usize) -> bool {
        self.data[Self::get_idx(self.n_points, (i1, i2, i3))]
    }
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
#[derive(Clone)]
struct Node {
    costs: CostMatrix,
    cost: f64,
    path: Option<Rc<PathPt>>,
    level: usize,
}

impl Node {
    fn new(
        costs: CostMatrix,
        cost: f64,
        prev: Option<Rc<Node>>,
        last_pt: usize,
        level: usize,
    ) -> Self {
        Self {
            costs,
            cost,
            prev,
            last_pt,
            level,
        }
    }
    fn to_path(&self, pts: Vec<Point>) -> Vec<Point> {
        let mut node = Rc::new(self.clone());
        let mut path = vec![pts[node.last_pt].clone()];
        while let Some(new_node) = &node.prev {
            path.push(pts[new_node.last_pt].clone());
            node = new_node.clone();
        }
        path.into_iter().rev().collect()
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

fn branch_and_bound(pts: Vec<Point>) -> Option<(Vec<Point>, f64)> {
    let angle_list = AngleOkList::new(&pts);
    let mut start_matrix = CostMatrix::new(&pts);
    let start_cost = start_matrix.reduce();
    let start_node = Node::new(start_matrix, start_cost, None, 0, 1);
    let mut queue = BinaryHeap::new();
    queue.push(Rc::new(start_node));
    let mut upper_bound = f64::MAX;
    let mut best_path: Option<Rc<Node>> = None;
    let mut max_len = 0;
    while let Some(node) = queue.pop() {
        if queue.len() > max_len {
            max_len = queue.len();
        }
        if queue.len() > 100_000 {
            break;
        }
        if node.cost > upper_bound {
            break;
        }
        if node.level == pts.len() + 1 {
            upper_bound = node.cost;
            best_path = Some(node);
            println!("found new best");
            break;
        }
        let p_pt = node.last_pt;
        queue.extend((0..node.costs.size).filter_map(|next_pt| {
            let (move_cost, mut new_costs) = node.costs.add_path(p_pt, next_pt)?;
            let cost = node.cost + move_cost + new_costs.reduce();
            if cost > upper_bound {
                return None;
            }
            let node = Rc::new(Node::new(
                new_costs,
                cost,
                Some(node.clone()),
                next_pt,
                node.level + 1,
            ));
            Some(node)
        }));
    }
    println!("max queue len: {}", max_len);
    Some((best_path?.to_path(pts), upper_bound))
}
fn draw_path(path: &Vec<Point>, image: &mut RgbaImage, color: [u8; 4], offset: f64) {
    let mut pts = path.into_iter();
    let mut p_pt = pts.next().unwrap();
    for pt in pts {
        draw_line_segment_mut(
            image,
            ((p_pt.0 + offset) as f32, p_pt.1 as f32),
            ((pt.0 + offset) as f32, pt.1 as f32),
            Rgba(color),
        );
        p_pt = pt;
    }
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

fn main() {
    let start = Instant::now();
    let size = 1000.0;
    let mut rng = rand::thread_rng();
    let points = /*load_pts("data/wenigerkrumm4.txt"); */get_points((size, size), 20, &mut rng);
    // let points = vec![
    //     Point(100.0, 200.0),
    //     Point(500.0, 200.0),
    //     Point(600.0, 300.0),
    // ];
    println!("Points: {:#?}", points);
    let do_bruteforce = false;
    let mut image = RgbaImage::from_fn(size as u32 * 2, size as u32, |_, _| {
        Rgba([0u8, 0u8, 0u8, 255u8])
    });
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
    let min_path_bnb = branch_and_bound(points).unwrap();
    println!(
        "bnb len: {}, cost: {}",
        get_len(&min_path_bnb.0),
        min_path_bnb.1
    );
    // println!("min_path: {:?}", min_path);

    let elapsed = start.elapsed();
    println!("took: {:?}", elapsed);
    draw_path(&min_path_bnb.0, &mut image, [0u8, 0u8, 255u8, 255u8], size);
    image.save("out.png").unwrap();
    println!("saved image");
}
