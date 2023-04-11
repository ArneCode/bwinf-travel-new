mod angle_list;
mod cost_matrix;
mod line;
mod path_finder;

use angle_list::AngleOkList;
use cost_matrix::CostMatrix;
use imageproc::drawing::draw_text_mut;
use line::Line;
use path_finder::PathFinder;
use rusttype::Font;
use rusttype::Scale;
use std::collections::HashMap;
use std::collections::HashSet;

use std::{f64::consts::PI, fs, time::Instant};

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

fn check_angles(pts: &Vec<Point>) {
    let mut p_pt: Option<Point> = None;
    let mut p_dir: Option<Dir> = None;
    let mut angles_ok = true;
    for pt in pts {
        if let Some(p_pt) = p_pt {
            let new_dir = p_pt.dir_to(pt);
            if let Some(p_dir) = p_dir {
                let angle = p_dir.angle_to(&new_dir);
                if !angle_ok(angle) {
                    angles_ok = false;
                }
            }
            p_dir = Some(new_dir);
        }
        p_pt = Some(*pt);
    }
    if angles_ok {
        println!("all angles ok");
    } else {
        println!("angles not ok");
    }
}
#[derive(Clone)]
struct Skip {
    end: usize,
    len: usize,
    cost: f64,
}
impl Skip {
    fn new(end: usize, len: usize, cost: f64) -> Self {
        Self { end, len, cost }
    }
}
enum NextPaths {
    Multiple(usize, Vec<usize>),
    Skip(Skip),
}
//function that makes a line of n points
fn find_shortest_paths(
    costs: &CostMatrix,
    lines: &Vec<Line>,
) -> Vec<(Vec<(usize, f64)>, Option<Skip>)> {
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
            [(0, 1), (1, 0)].into_iter().map(|(a, b)| {
                (
                    l.ends[a],
                    Skip::new(l.ends[b], l.pts.len(), l.get_cost(costs)),
                )
            })
        })
        .collect();
    let n_pts = costs.size;
    (0..n_pts)
        .map(|start| {
            let skip = line_skips.get(&start).cloned();
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
            //find average cost
            let avg_cost = nexts.iter().map(|(_b, cost)| cost).sum::<f64>() / nexts.len() as f64;
            //calculate the largest index that is still ok
            let mut max_idx = nexts.len();
            for (i, (_b, cost)) in nexts.iter().enumerate() {
                if cost > &avg_cost {
                    max_idx = i;
                    break;
                }
            }
            //remove all that are too expensive
            //nexts.truncate(max_idx);
            //let nexts = nexts.into_iter().map(|(b, _)| b).collect();
            (nexts, skip)
        })
        .collect::<Vec<_>>()
}
fn find_min_cost(
    short_paths: &[(Vec<(usize, f64)>, Option<Skip>)],
    start_pt: usize,
    lines: &[Line],
) -> (f64, Vec<(usize, f64)>, Vec<(f64, f64)>) {
    let line_pts: HashSet<usize> = lines
        .iter()
        .flat_map(|l| -> Vec<usize> { (l.pts[1..l.pts.len() - 1]).to_vec() })
        .collect();
    //find the first and second cost for every point
    let costs = short_paths
        .iter()
        .map(|(nexts, _)| -> (f64, f64) {
            //return the first and second element, and thus the first and second lowest costs
            let costs = nexts.iter().map(|(_b, cost)| cost).collect::<Vec<_>>();
            (*costs[0], *costs[1])
        })
        .collect::<Vec<_>>();

    //find the point with the highest second cost (not in the line or the start point)
    let max_second_costs = {
        let mut costs = costs
            .iter()
            .enumerate()
            .filter(|(i, _)| !line_pts.contains(i) && *i != start_pt)
            .map(|(i, (_a, b))| (i, *b))
            .collect::<Vec<_>>();
        //sort by second cost descending
        costs.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        costs
    };
    let max_idx = max_second_costs[0].0;
    let mut cost = 0.0;
    for (i, (first, second)) in costs.iter().enumerate() {
        if line_pts.contains(&i) {
            continue;
        }
        cost += first;
        if i != start_pt && i != max_idx {
            cost += second;
        }
    }
    cost /= 2.0;
    println!("cost: {}", cost);
    (cost, max_second_costs, costs)
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
    let mut finder = PathFinder::new(costs, angle_list);
    finder.run()
}

fn find_nexts(
    path: &[usize],
    free_pts: &[bool],
    short_paths: &[(Vec<usize>, Option<usize>)],
    angle_list: &AngleOkList,
) -> Vec<usize> {
    let p_pt = if path.len() > 1 {
        path.get(path.len() - 2).cloned()
    } else {
        None
    };
    let curr_pt = path[path.len() - 1];
    let next_pts = short_paths[curr_pt].0.clone();
    let _next_pts = next_pts
        .iter()
        .filter_map(|&pt| {
            //filter out points that are not free or have a bad angle
            if free_pts[pt] && (p_pt.is_none() || angle_list.is_ok(p_pt.unwrap(), curr_pt, pt)) {
                Some(pt)
            } else {
                None
            }
        })
        .map(|_pt| { //calculate the cost of the pt
        })
        .collect::<Vec<_>>();
    todo!()
}
fn idxs_to_pts(idxs: Vec<usize>, pts: &[Point]) -> Vec<Point> {
    idxs.into_iter().map(|i| pts[i]).collect()
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

//reuse the path from find_n_n in find_path_jump

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
                let width: i32 = 3;
                for yoff in (-width)..(width) {
                    let yoff = yoff as f32;
                    for xoff in (-width)..(width) {
                        let xoff = xoff as f32;
                        //draw the line
                        draw_line_segment_mut(
                            image,
                            ((p_pt.0) as f32 + xoff, p_pt.1 as f32 + yoff),
                            ((pt.0) as f32 + xoff, pt.1 as f32 + yoff),
                            Rgba(color),
                        );
                    }
                }
            }
        }
        //draw the point
        draw_filled_circle_mut(image, (pt.0 as i32, pt.1 as i32), 15, Rgba(pt_color));
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
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    ///ob die linien angezeigt werden sollen
    #[clap(short, long)]
    lines: bool,
    ///ob der kürzeste Pfad berechnet werden soll
    #[clap(long)]
    no_calc: bool,
    ///ob die Punkte angezeigt werden sollen
    #[clap(short, long)]
    points: bool,
    ///der Pfad zu den Punkten
    #[clap(required = true)]
    path: String,
}

fn main() {
    let args = Args::parse();
    println!("{:?}", args);
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

    //let mut points = get_points((size, size), 7, &mut rng);
    //let points = make_line(40, Point(0.0,0.0), Dir(1.0, 0.0));
    //println!("points len: {}", points.len());
    let mut points = load_pts(&args.path);
    //points.shuffle(&mut rng);
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
    if args.points {
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
    if args.lines {
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

    if !args.no_calc {
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
