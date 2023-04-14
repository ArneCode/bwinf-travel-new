mod angle_list;
mod cost_list;
mod line;
mod path_finder;

use angle_list::AngleOkList;
use cost_list::CostList;
use imageproc::drawing::draw_text_mut;
use line::Line;
use path_finder::PathFinder;
use rusttype::Font;
use rusttype::Scale;
use std::collections::HashMap;
use std::collections::HashSet;

use std::fs::File;
use std::io::Write;
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
    print!("Überprüfe Winkel... ");
    if angles_ok {
        println!("alle Winkel in Ordnung");
    } else {
        println!("Winkel nicht in Ordnung");
    }
}
/// Speichert Sprünge
#[derive(Clone)]
pub struct Skip {
    end: usize,            //Ende der Linie
    second_pt: usize,      //Der Punkt nach dem Anfang der Linie
    penultimate_pt: usize, //Der Punkt vor dem Ende der Linie
    cost: f64,             //Wie lange die Linie ist
}
impl Skip {
    fn new(end: usize, second_pt: usize, penultimate_pt: usize, cost: f64) -> Self {
        Self {
            end,
            second_pt,
            penultimate_pt,
            cost,
        }
    }
}
enum NextPaths {
    Multiple(usize, Vec<usize>),
    Skip(Skip),
}
fn get_skips(costs: &CostList, lines: &Vec<Line>) -> Vec<Option<Skip>> {
    let mut skips = vec![None; costs.size];
    for line in lines {
        for (skip_start, skip_end) in [(0, 1), (1, 0)].iter() {
            let second_pt = line.seconds[*skip_start];
            let penultimate_pt = line.seconds[*skip_end];
            let skip = Skip::new(
                line.ends[*skip_end],
                second_pt,
                penultimate_pt,
                line.get_cost(costs),
            );
            skips[line.ends[*skip_start]] = Some(skip);
        }
    }
    skips
}
//function that makes a line of n points
fn find_nearest_pts(costs: &CostList, lines: &Vec<Line>) -> Vec<Vec<(usize, f64)>> {
    let line_pts: HashSet<usize> = lines
        .iter()
        .flat_map(|l| -> Vec<usize> { (l.pts[1..l.pts.len() - 1]).to_vec() })
        .collect();
    let mut line_len = 0;
    for line in lines {
        line_len += line.pts.len();
    }
    let n_pts = costs.size;
    let result = (0..n_pts)
        .map(|start| {
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
            nexts
        })
        .collect::<Vec<_>>();
    result
}
fn find_min_cost(
    short_paths: &[Vec<(usize, f64)>],
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
        .map(|nexts| -> (f64, f64) {
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
    (cost, max_second_costs, costs)
}
//fügt die linien wieder in die liste ein
fn insert_lines(path: &Vec<usize>, lines: Vec<Line>) -> Vec<usize> {
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
        new_path.push(*pt);
    }
    new_path
}

pub fn idxs_to_pts(idxs: Vec<usize>, pts: &[Point]) -> Vec<Point> {
    idxs.into_iter().map(|i| pts[i]).collect()
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
    args: &Args,
    //offset: f64,
    show_labels: bool,
    show_connections: bool,
) {
    //get the size of the image
    let size = image.dimensions();
    let size = (size.0 as f64, size.1 as f64);
    let (min, max) = pt_bounds;
    let height: f32 = 2.0 * args.pt_size as f32; // to get 80 chars across (fits most terminals); adjust as desired

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
                let width = args.line_width;
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
        draw_filled_circle_mut(
            image,
            (pt.0 as i32, pt.1 as i32),
            args.pt_size,
            Rgba(pt_color),
        );
        let label = i.to_string(); //+ ": " + &angle.to_string() + "°";
                                   //println!("label for {:?}: {}", pt, label);
        if show_labels {
            draw_text_mut(
                image,
                Rgba(pt_color),
                pt.0 as i32,
                pt.1 as i32 - args.pt_size * 2,
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
//saves points to a file
fn save_pts(path: &str, pts: &[Point]) -> Result<(), std::io::Error> {
    let mut s = String::new();
    for pt in pts {
        s += &pt.0.to_string();
        s += " ";
        s += &pt.1.to_string();
        s += "\n";
    }
    let mut file = File::create(path)?;
    file.write_all(s.as_bytes())?;
    Ok(())
}

//arguments
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
pub struct Args {
    ///Ob die linien angezeigt werden sollen
    #[arg(long)]
    show_lines: bool,
    ///Ob Linen beim finden des Pfads benutzt werden sollen
    #[arg(long)]
    dont_use_lines: bool,
    ///Ob der Pfad berechnet werden soll
    #[arg(long)]
    no_calc: bool,
    ///Ob die Punkte angezeigt werden sollen
    #[arg(long)]
    show_points: bool,
    ///Die Punkte-Datei
    #[clap(required = true)]
    path: String,
    ///Die Linien-Mindestlänge
    #[arg(long, default_value = "5")]
    line_min: usize,
    ///Die Bildgröße
    #[arg(long, default_value = "4000")]
    img_size: u32,
    ///Punktgröße im Bild
    #[arg(long, default_value = "15")]
    pt_size: i32,
    ///Linienbreite
    #[arg(long, default_value = "3")]
    line_width: i32,
    ///Wo das Bild gespeichert werden soll
    #[arg(long, default_value = "out.png")]
    img_path: String,
    ///Wo der Pfad gespeichert werden soll
    #[arg(long, default_value = "out.txt")]
    out_path: String,
    ///Wie lange der Pfad berechnet werden soll
    #[arg(long, default_value = "10000000")]
    max_iter: u128,
}

fn main() {
    let args = Args::parse();
    let start = Instant::now();
    let size = args.img_size;
    let mut rng = rand::thread_rng();
    let mut points = load_pts(&args.path);
    println!("Punkte aus Datei geladen");
    if false {
        points.shuffle(&mut rng);
    }
    let pt_bounds = calc_bounds(&points);
    //points.shuffle(&mut rng);
    let _pts_len = points.len();

    let mut image = RgbaImage::from_fn(size as u32, size as u32, |_, _| {
        Rgba([255u8, 255u8, 255u8, 255u8])
    });
    if args.show_points {
        draw_path(
            &points,
            &mut image,
            //size:
            pt_bounds,
            [0u8, 0u8, 255u8, 255u8],
            [0u8, 0u8, 255u8, 255u8],
            &args,
            false,
            false,
        );
    }
    if args.show_lines {
        let costs = CostList::new(&points);
        let angle_list = AngleOkList::new(&points);
        let lines = find_lines(&costs, &angle_list, args.line_min);
        for line in lines {
            println!("line: {:?}", line);
            let line = idxs_to_pts(line.pts, &points);
            draw_path(
                &line,
                &mut image,
                pt_bounds,
                [255u8, 0u8, 0u8, 255u8],
                [255u8, 0u8, 0u8, 255u8],
                &args,
                true,
                true,
            );
        }
    }

    if !args.no_calc {
        println!("Suche Weg...");
        let mut finder = PathFinder::new(&points, &args);
        if let Some(min_path) = finder.find() {
            let len = get_len(&min_path);
            println!("Weg gefunden, Länge: {}", len);
            check_angles(&min_path);

            let elapsed = start.elapsed();
            println!("finden des Wegs hat {:?} gedauert", elapsed);
            draw_path(
                &min_path,
                &mut image,
                //size:
                pt_bounds,
                [0u8, 0u8, 255u8, 255u8],
                [255u8, 0u8, 255u8, 255u8],
                &args,
                true,
                true,
            );
            save_pts(&args.out_path, &min_path).unwrap();
        } else {
            println!("konnte keinen Weg finden");
        }
    }
    image.save(&args.img_path).unwrap();
    println!("Bild gespeichert");
}
