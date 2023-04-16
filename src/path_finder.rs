use std::{collections::HashSet, time::Instant};

use crate::{
    angle_list::AngleOkList,
    cost_list::CostList,
    find_min_cost, find_nearest_pts, get_skips, idxs_to_pts, insert_lines,
    line::{find_lines, Line},
    Args, Point, Skip,
};
//Ist für die Berechnung des Pfades zuständig
//Da diese Klasse für viele Sachen zuständig ist,
//ist die Anzahl der Member-Variablen relativ groß.
//Ich habe jedoch festgestellt, dass das trennen der Klasse
//in mehrere Verschiedene Einheiten zu mehr Komplexität führt,
//da verschiedene Werte oft an vielen Stellen benötigt werden,
//weshalb die Klasse jetzt etwas mehr Klassenvariablen als ideal hat.
pub struct PathFinder<'a> {
    points: &'a Vec<Point>,  //alle Punkte
    args: &'a Args,          //Parameter
    n_pts: usize,            //Anzahl der Punkte (ohne Linienpunkte)
    angle_list: AngleOkList, //Welche Punkte mit welchem Punkt verbunden werden können

    cost_list: CostList,                 //Abstand zwischen allen Punkten
    min_costs: Vec<(f64, f64)>,          //Mindestabstände für jeden Punkt
    nearest_pts: Vec<Vec<(usize, f64)>>, //nächste Punkte für jeden Punkt
    //Die Punkte sortiert nach zweitbestem Abstand (siehe Branch and Bound Umsetzung)
    max_second_costs: Vec<(usize, f64)>,
    start_pt: usize, //Startpunkt

    lines: Vec<Line>,                   //gefundene Linien
    available_skips: Vec<Option<Skip>>, //von welchen Punkten aus gesprungen werden kann

    free_pts: Vec<bool>, //welche Punkte noch frei sind

    path: Vec<usize>, //aktueller Pfad
    //speichert für jeden Punkt im Pfad,
    //der wievielt nähste Punkt als nächstes drankommt
    idxs: Vec<usize>,
    prev_costs: Vec<f64>,          //Die vorherigen Kosten
    prev_end_idxs: Vec<usize>,     //Die vorherigen Endpunkte (für jeden Punkt im Pfad)
    prev_skips: Vec<Option<Skip>>, //Ob die vorherigen Punkte übersprungen wurden

    cost: f64,           //aktuelle untere Grenze
    curr_end_idx: usize, //der wievielte Endpunkt der aktuelle ist
    //wie oft die Funktion run() aufgerufen wurde,
    //wird benutzt um einen neuen Startpunkt zu bestimmen
    n_runs: usize,

    went_back: bool, //ob schon zurückgegangen wurde
}
impl<'a> PathFinder<'a> {
    /// erzeugt ein neues PathFinder Objekt, berechnet dabei viele Werte vorab
    pub fn new(points: &'a Vec<Point>, args: &'a Args) -> Self {
        let costs = CostList::new(points);
        let angle_list = AngleOkList::new(&points, &args);
        let lines = if !args.dont_use_lines {
            find_lines(&costs, &angle_list, args.line_min)
        } else {
            vec![]
        };

        let nearest_pts = find_nearest_pts(&costs, &lines);
        let available_skips = get_skips(&costs, &lines);
        let mut n_pts = costs.size;
        let line_pts: HashSet<usize> = lines
            .iter()
            .flat_map(|l| -> Vec<usize> { (l.pts[1..l.pts.len() - 1]).to_vec() })
            .collect();
        //find the first and second cost for every point
        let min_costs = nearest_pts
            .iter()
            .map(|nexts| -> (f64, f64) {
                //return the first and second element, and thus the first and second lowest costs
                let costs = nexts.iter().map(|(_b, cost)| cost).collect::<Vec<_>>();
                (*costs[0], *costs[1])
            })
            .collect::<Vec<_>>();
        //find the point with the highest second cost (not in the line or the start point)
        let max_second_costs = {
            let mut costs = min_costs
                .iter()
                .enumerate()
                .filter(|(i, _)| !line_pts.contains(i))
                .map(|(i, (_a, b))| (i, *b))
                .collect::<Vec<_>>();
            //sort by second cost descending
            costs.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
            costs
        };
        let start_pt_n = args.start_pt_n.unwrap_or(0);
        //find the start point
        let start_pt = max_second_costs[start_pt_n].0;
        println!("start_pt: {}", start_pt);
        let curr_end_idx = if start_pt_n == 0 { 1 } else { 0 };
        let cost = {
            let max_idx = max_second_costs[curr_end_idx].0;
            let mut cost = 0.0;
            for (i, (first, second)) in min_costs.iter().enumerate() {
                if line_pts.contains(&i) {
                    continue;
                }
                cost += first;
                if i != start_pt && i != max_idx {
                    cost += second;
                }
            }
            cost /= 2.0;
            cost
        };

        let mut free_pts = vec![true; n_pts];
        free_pts[start_pt] = false;
        for line in &lines {
            n_pts -= line.pts.len() - 2;
        }

        let went_back = false;

        let path = vec![start_pt];
        let idxs = vec![0];
        let prev_costs = vec![cost];
        let prev_skips = vec![None];
        let prev_end_idxs = vec![0];
        let n_runs = 0;
        Self {
            points,
            args,
            angle_list,
            lines,
            nearest_pts,
            available_skips,
            path,
            idxs,
            prev_costs,
            start_pt,
            prev_skips,
            n_pts,
            max_second_costs,
            curr_end_idx,
            prev_end_idxs,
            min_costs,
            cost_list: costs,
            cost,
            free_pts,
            n_runs,
            went_back,
        }
    }
    fn reset(&mut self) {
        println!("reset mit Startpunkt: {}", self.start_pt);
        self.path = vec![self.start_pt];
        self.idxs = vec![0];
        self.cost = self.prev_costs[0];
        self.prev_costs = vec![self.cost];
        self.prev_skips = vec![None];
        self.prev_end_idxs = vec![0];
        let prev_end = self.max_second_costs[self.curr_end_idx];
        self.curr_end_idx = 0;
        let curr_end = self.max_second_costs[self.curr_end_idx];
        self.cost += curr_end.1 - prev_end.1;
        self.went_back = false;
        self.free_pts = vec![true; self.free_pts.len()];
        self.free_pts[self.start_pt] = false;
    }
    fn next_start(&mut self) {
        self.n_runs += 1;
        self.start_pt = self.max_second_costs[self.n_runs].0;
        self.reset()
    }
    fn get_prev_pt(&self) -> Option<usize> {
        let level = self.path.len() - 1;
        if let Some(skip) = &self.prev_skips[level] {
            Some(skip.penultimate_pt)
        } else if level > 0 {
            Some(self.path[level - 1])
        } else {
            None
        }
    }
    ///geht einen Schritt zum vorherigen Punkt zurück
    fn backtrack(&mut self, curr_pt: usize) {
        //markiert den nächsten Punkt als frei
        self.free_pts[curr_pt] = true;
        //entfernt den letzten Punkt aus dem Pfad
        self.path.pop().unwrap();
        //entfernt den letzten Index aus den Indexen
        self.idxs.pop().unwrap();

        // falls man sich schon am Anfang befindet kann nicht weiter zurückgegangen werden
        if self.path.len() == 0 {
            return;
        }

        // Setzt die aktuelle untere Grenze zurück
        self.cost = self.prev_costs.pop().unwrap();
        // Setzt den aktuellen Endpunkt zurück
        self.curr_end_idx = self.prev_end_idxs.pop().unwrap();

        self.prev_skips.pop().unwrap();
        self.went_back = true;
    }
    /// berechnet neue mindestkosten
    pub fn update_costs(&mut self, curr_pt: usize, next_pt: usize) -> f64 {
        let mut cost = 0.0;
        //Der aktuelle Endpunkt
        let curr_last = self.max_second_costs[self.curr_end_idx];
        //falls der aktuelle Punkt der Endpunkt ist
        //wird nach einem neuen Endpunkt gesucht
        if curr_pt == curr_last.0 {
            //sucht den nächsten freien Endpunkt
            let new_last = loop {
                self.curr_end_idx += 1; //geht zum nächsten Endpunkt
                                        //in max_second_costs wurden mögliche Endpunkte
                                        //sortiert nach tauglichkeit gespeichert
                let new_last = self.max_second_costs[self.curr_end_idx];
                if self.free_pts[new_last.0] && new_last.0 != self.start_pt {
                    //falls der Endpunkt frei ist wird er genommen
                    break new_last;
                }
            };
            let prev_last_cost = curr_last.1;
            let new_last_cost = new_last.1;
            cost += (prev_last_cost - new_last_cost) / 2.0;
        }
        //falls der vorherige punkt der Startpunkt ist
        let min_prev = if self.path.len() == 1 {
            //wird für den vorherigen Punkt die kürzere Strecke abgezogen
            self.min_costs[curr_pt].0
        } else {
            //sonst die längere
            self.min_costs[curr_pt].1
        };
        //Die kürzere Strecke zum nächsten Punkt:
        let min_next = self.min_costs[next_pt].0;
        cost -= (min_prev + min_next) / 2.0;
        cost
    }
    /// findet den nächst-nähsten punkt, der passt
    fn find_next_nearest(&mut self, curr_pt: usize) -> Option<(usize, f64)> {
        let level = self.path.len() - 1; //gibt an, wie weit man sich im Pfad befindet
                                         //der vorherige punkt, ist None, wenn der aktuelle punkt der erste ist
        let p_pt = self.get_prev_pt();
        //der wivielt-nähste punkt als nächstes genommen wird
        let curr_idx = self.idxs.get_mut(level).unwrap();
        //die liste der nähsten punkte
        let next_pts = &self.nearest_pts[curr_pt];
        //In rust gibt es kein do-while. Um trotzdem zuerst einmal durchzulaufen und dann zu prüfen,
        //ob abgebrochen werden soll, wird ein loop mit break benutzt
        loop {
            // falls alle nächsten punkte ausgeschlossen wurden
            // wird wieder zurückgegangen, da kein nächster Punkt gefunden wurde
            if *curr_idx >= next_pts.len() {
                return None;
            }
            let (next_pt, cost) = next_pts[*curr_idx];
            //falls der näheste punkt schon benutzt wurde oder der winkel nicht passt
            if !self.free_pts[next_pt]
                //überprüft den winkel nur, wenn es einen vorherigen punkt gib
                //das heißt, dass der aktuelle punkt nicht der startpunkt ist
                || (p_pt.is_some() && !self.angle_list.is_ok(p_pt.unwrap(), curr_pt, next_pt))
            {
                *curr_idx += 1; //wird der nächstbeste punkt genommen
            } else {
                //falls der nächste punkt passt, wird er zurückgegeben
                break Some((next_pt, cost));
            }
        }
    }
    /// findet den nächsten punkt, entweder den nächsten nähsten oder einen Sprung
    pub fn find_nexts(&mut self, curr_pt: usize) -> Option<(usize, f64, Option<Skip>)> {
        let level = self.path.len() - 1; //gibt an, wie weit man sich im Pfad befindet
                                         //der vorherige punkt, ist None wenn der aktuelle punkt der erste ist
        let p_pt = self.get_prev_pt();
        //der wivielt-nähste punkt als nächstes genommen wird,
        //wird benutzt, um zu prüfen, ob auf diesen punkt schon zurückgesprungen wurde
        let curr_idx = *self.idxs.get(level).unwrap();

        let prev_skip = &self.prev_skips[level];
        let skip = &self.available_skips[curr_pt];
        //falls der vorherige Schritt ein Sprung war, ist der aktuelle Punkt das Ende der Linie.
        //Wieder zu springen würde also bedeuten wieder an den Anfang der Linie zu springen,
        //wo man gerade hergekommen ist.
        if skip.is_some() && !prev_skip.is_some() {
            let skip = skip.clone().unwrap();
            if curr_idx == 0 // falls nicht auf den Punkt zurückgesprungen wurde
                // wenn der vorherige punkt nicht None ist wird der winkel geprüft
                // hier wird als dritter Punkt der zweite Punkt der Linie genommen
                && (p_pt.is_none() || self.angle_list.is_ok(p_pt.unwrap(), curr_pt, skip.second_pt))
            {
                Some((skip.end, skip.cost, Some(skip)))
            } else {
                //Man befindet sich am Anfang der Linie und kann nicht springen,
                //also wird zurückgegangen
                None
            }
        } else {
            //Falls man sich nicht am Anfang einer Linie befindet,
            //wird der nächste Punkt normal gesucht
            let (next_pt, cost) = self.find_next_nearest(curr_pt)?;
            Some((next_pt, cost, None))
        }
    }
    /// findet mögliche Wege
    pub fn find(&mut self) -> Option<Vec<Point>> {
        let start = Instant::now();
        let mut best_start_n = None;
        let mut best_start = None;
        let mut best_cost = f64::INFINITY;
        loop {
            if let Some(result) = self.run() {
                if self.args.search_start {
                    let cost = self.cost_list.calc_len(&result);
                    if cost < best_cost {
                        best_cost = cost;
                        best_start_n = Some(self.n_runs);
                        best_start = Some(self.start_pt);
                        println!(
                            "\nNeuer bester Startpunkt, Nummer: {}, Punkt: {}\n",
                            self.n_runs, self.start_pt
                        );
                    } else {
                        println!("\nStartpunkt Nummer {} ist nicht besser als der bisher beste Startpunkt\n", self.n_runs);
                    }
                    if self.n_runs >= self.n_pts - 1 {
                        if let Some(best_start_n) = best_start_n {
                            println!(
                                "Bester Startpunkt ist Startpunkt Nummer {}: {}, Cost: {}",
                                best_start_n,
                                best_start.unwrap(),
                                best_cost
                            );
                        } else {
                            println!("Kein Startpunkt gefunden");
                        }
                    } else {
                        self.next_start();
                        continue;
                    }
                }
                println!("Pfad gefunden");
                println!("\tDauer (ohne Vorberechnungen): {:?}", start.elapsed());
                //Indexe werden in Punkte umgewandelt
                let points = idxs_to_pts(result, self.points);
                return Some(points);
            }
            if self.n_runs >= self.n_pts - 1 {
                if self.args.search_start {
                    if let Some(best_start_n) = best_start_n {
                        println!(
                            "Bester Startpunkt ist Startpunkt Nummer {}: {}, Cost: {}",
                            best_start_n,
                            best_start.unwrap(),
                            best_cost
                        );
                    } else {
                        println!("Kein Startpunkt gefunden");
                    }
                }
                println!("kein Pfad gefunden");
                return None;
            }
            println!("kein Pfad gefunden, nächster Startpunkt");
            self.next_start();
        }
    }
    fn run(&mut self) -> Option<Vec<usize>> {
        let mut i: u128 = 0;
        let mut min_path: Option<Vec<usize>> = None;
        let mut min_cost = f64::INFINITY;
        let mut n_high: u128 = 0;
        let mut min_level = usize::MAX;
        //self.cost = 0.0;

        'main_loop: loop {
            i += 1;
            if i > self.args.max_iter && min_path.is_some() {
                println!("max_iter erreicht");
                println!(
                    "tiefstes Level auf das zurückgegangen wurde: {}/{}, Abgeschnittene Pfade: {}/{} = {:.3}%",
                    min_level,
                    self.n_pts,
                    n_high,
                    i,
                    n_high as f64 / i as f64 * 100.0
                );
                return min_path;
            }
            if i % 100_000_000 == 0 {
                println!(
                    "i: {}, Abgeschnittene Pfade: {:.3}%",
                    i,
                    n_high as f64 / i as f64 * 100.0
                );
            }
            if i > 1000_000 && min_path.is_none() {
                return None;
            }
            if self.path.is_empty() {
                if min_path.is_some() {
                    println!("Alle möglichen Pfade durchprobiert");
                    return min_path;
                }
                return None;
            }
            let level = self.path.len() - 1;

            if self.went_back && level < min_level {
                min_level = level;
            }
            let level = self.path.len() - 1;

            //self.idxs.push(curr_idx + 1);
            let curr_pt = self.path[level];

            let (next_pt, move_cost, did_skip) = if let Some(result) = self.find_nexts(curr_pt) {
                result
            } else {
                self.backtrack(curr_pt);
                continue 'main_loop;
            };
            self.prev_costs.push(self.cost);
            self.prev_end_idxs.push(self.curr_end_idx);
            self.cost += move_cost + self.update_costs(curr_pt, next_pt);

            if self.cost > min_cost {
                n_high += 1;
                self.prev_costs.pop();
                self.prev_end_idxs.pop();
                self.backtrack(curr_pt);
                continue 'main_loop;
            }

            //Vorbereiten zum nächsten Punkt weiterzugehen:

            // erhöht den index des aktuellen punkts um 1,
            // damit beim nächsten mal der nächste punkt genommen wird
            let curr_idx = self.idxs.get_mut(level).unwrap();
            *curr_idx += 1;
            //fügt 0 als index für den nächsten punkt hinzu
            self.idxs.push(0);

            //Speichert den nächsten punkt in dem Pfad
            self.path.push(next_pt);
            //markiert den nächsten punkt als belegt
            self.free_pts[next_pt] = false;

            self.prev_skips.push(did_skip);

            if self.path.len() == self.n_pts {
                let full_path = insert_lines(&self.path, self.lines.clone());
                if self.cost < min_cost {
                    //self.cost -= cost;
                    let n_high_per = n_high as f64 / i as f64;
                    println!(
                        "neue untere Grenze: {}, Abgeschnittene Pfade: {}/{} = {:.3}%",
                        self.cost,
                        n_high,
                        i,
                        n_high_per * 100.0,
                        //self.prev_costs
                    );
                    min_cost = self.cost;
                    min_path = Some(full_path.clone());
                    if self.args.stop_on_found {
                        println!("--stop_on_found benutzt, beende Suche");
                        return min_path;
                    }
                }
                self.backtrack(next_pt);

                continue 'main_loop;
            }
        }
    }
}
