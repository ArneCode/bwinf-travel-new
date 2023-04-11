use crate::{
    angle_list::AngleOkList,
    cost_matrix::CostMatrix,
    find_min_cost, find_shortest_paths, insert_lines,
    line::{find_lines, Line},
    Skip,
};

pub struct PathFinder {
    angle_list: AngleOkList,
    lines: Vec<Line>,
    short_paths: Vec<(Vec<(usize, f64)>, Option<Skip>)>,
    n_pts: usize,
    max_second_costs: Vec<(usize, f64)>,
    curr_last_idx: usize,
    costs: Vec<(f64, f64)>,
    path: Vec<usize>,
    idxs: Vec<usize>,
    cost: f64,
    prev_costs: Vec<f64>,
    free_pts: Vec<bool>,
    skips: Vec<bool>,
    went_back: bool,
}
impl PathFinder {
    pub fn new(costs: &CostMatrix, angle_list: &AngleOkList) -> Self {
        let lines = find_lines(costs, angle_list);
        //lines = vec![];
        println!(
            "lines: {:?}",
            lines.iter().map(|l| l.ends).collect::<Vec<_>>()
        );
        //findet die nähsten punkte zu jedem punkt
        let short_paths = find_shortest_paths(costs, &lines);

        let start_pt = if lines.is_empty() {
            1
        } else {
            lines[0].ends[0]
        };
        let mut n_pts = costs.size;
        let (cost, max_second_costs, costs) = find_min_cost(&short_paths, start_pt, &lines);
        //println!("cost: {}", cost);
        let mut free_pts = vec![true; n_pts];
        free_pts[start_pt] = false;
        for line in &lines {
            n_pts -= line.pts.len() - 2;
        }
        let path = vec![start_pt];
        let idxs = vec![0];
        let added_costs = vec![];
        let cost = cost;
        let skips = vec![false];
        let curr_last = 0;
        let went_back = false;
        Self {
            angle_list: angle_list.clone(),
            lines,
            short_paths,
            n_pts,
            max_second_costs,
            curr_last_idx: curr_last,
            costs,
            path,
            idxs,
            cost,
            prev_costs: added_costs,
            free_pts,
            skips,
            went_back,
        }
    }
    fn backtrack(&mut self, curr_pt: usize) {
        self.went_back = true;
        self.free_pts[curr_pt] = true;
        self.path.pop();
        self.skips.pop();
        self.cost = self.prev_costs.pop().unwrap_or(0.0);
        if self.curr_last_idx > 0 {
            let prev_last_pt = self.max_second_costs[self.curr_last_idx - 1].0;
            if prev_last_pt == curr_pt {
                self.curr_last_idx -= 1;
            }
        }
    }
    pub fn run(&mut self) -> Option<Vec<usize>> {
        let mut i: u128 = 0;
        let mut n_back = 0;
        let mut min_path: Option<Vec<usize>> = None;
        let mut min_cost = f64::INFINITY;
        let mut n_high = 0;
        let mut min_level = usize::MAX;

        'main_loop: loop {
            i += 1;
            if i > 100_000_000 && min_path.is_some() {
                println!("lowest level: {}/{}", min_level, self.n_pts);
                return min_path;
            }
            if i % 100_000_000 == 0 {
                println!("i: {}, n_back: {}, idxs: {:?}", i, n_back, self.idxs);
            }
            //*
            if i > (self.n_pts.pow(4)) as u128 {
                //println!("this is taking too long, abort");
                //return None;
            }
            //*/
            // println!("path: {path:?}, idxs: {idxs:?}");
            if self.idxs.is_empty() {
                if min_path.is_some() {
                    return min_path;
                }
                panic!(
                    "idxs is empty, idxs: {:?}, path: {:?}, i: {}",
                    self.idxs, self.path, i
                );
            }
            let level = self.idxs.len() - 1;
            if self.went_back && level < min_level {
                min_level = level;
            }
            let curr_pt = self.path[level];
            let prev_was_skip = self.skips[level];

            //println!("level: {}, i: {}", level, i);
            let p_pt = if level > 0 {
                self.path.get(level - 1).cloned()
            } else {
                None
            };
            let mut curr_idx = self.idxs.pop().unwrap();

            let (next_pts, skip) = &self.short_paths[curr_pt];

            let mut did_skip = false;
            let (next_pt, move_cost) = if skip.is_some() && !prev_was_skip {
                //println!("skip: {:?}, i: {}, ", skip, i);
                let skip = skip.clone().unwrap();
                if curr_idx == 0
                    && (p_pt.is_none() || self.angle_list.is_ok(p_pt.unwrap(), curr_pt, skip.end))
                {
                    did_skip = true;

                    (skip.end, skip.cost)
                } else {
                    self.backtrack(curr_pt);
                    continue 'main_loop;
                }
            } else {
                //println!("no skip");
                //skips.push(false);
                loop {
                    // falls alle nächsten punkte ausgeschlossen wurden
                    // wird wieder zurückgegangen
                    if curr_idx >= next_pts.len() {
                        n_back += 1;

                        //println!("backtracking, len: {}, max_idx: {}", i, max_idx);
                        self.backtrack(curr_pt);
                        continue 'main_loop;
                    }
                    let (next_pt, cost) = next_pts[curr_idx];
                    //falls der näheste punkt schon benutzt wurde oder der winkel nicht passt
                    if !self.free_pts[next_pt]
                        || (p_pt.is_some()
                            && !self.angle_list.is_ok(p_pt.unwrap(), curr_pt, next_pt))
                    {
                        curr_idx += 1; //wird der nächstbeste punkt genommen
                    } else {
                        break (next_pt, cost);
                    }
                }
            };
            //update costs
            let curr_last = self.max_second_costs[self.curr_last_idx];
            let mut cost = move_cost;
            if curr_pt == curr_last.0 {
                let prev_last_cost = curr_last.1;
                self.curr_last_idx += 1;
                let new_last_cost = self.max_second_costs[self.curr_last_idx].1;
                cost += (prev_last_cost - new_last_cost) / 2.0;
            }
            let min_0 = if level == 0 {
                //get min cost for curr_pt
                self.costs[curr_pt].0
            } else {
                self.costs[curr_pt].1
            };
            let min_1 = self.costs[next_pt].0;
            cost -= (min_0 + min_1) / 2.0;
            //self.prev_costs.push(cost);
            self.cost += cost;
            if self.cost > min_cost {
                n_high += 1;
                //println!("cost too high, i: {}, cost: {}, min_cost: {}", i, self.cost, min_cost);
                self.backtrack(curr_pt);
                continue 'main_loop;
            }

            self.path.push(next_pt);
            self.skips.push(did_skip);
            if self.path.len() == self.n_pts {
                //println!("path len: {}, skip len: {}", path.len(), skips.len());
                //println!("found new path: {:?}", path);
                let full_path = insert_lines(self.path.clone(), self.lines.clone());

                if self.cost < min_cost {
                    let n_high_per = n_high as f64 / i as f64;
                    println!(
                        "new min cost: {}, n_high: {}/{} = {:.3}%",
                        self.cost,
                        n_high,
                        i,
                        n_high_per * 100.0
                    );
                    min_cost = self.cost;
                    min_path = Some(full_path.clone());
                }
                self.path.pop();
                self.skips.pop();
                self.backtrack(curr_pt);
                //self.backtrack(curr_pt);
                //println!("backtracking because of new path, i: {}, curr_pt: {}, path: {:?}", i, curr_pt, &path[path.len() - 4..]);

                continue 'main_loop;
            }

            //println!("moving to next point: {}", next_pt);
            //moving to the next point
            self.free_pts[next_pt] = false;
            self.idxs.push(curr_idx + 1);
            self.idxs.push(0);
            self.prev_costs.push(self.cost);
        }
        // todo!()
    }
}
