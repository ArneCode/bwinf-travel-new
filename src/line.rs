use std::collections::{HashMap, LinkedList};

use crate::{angle_list::AngleOkList, CostList};

#[derive(Debug, Clone)]
pub struct Line {
    pub ends: [usize; 2], // Start und Endpunkt
    pub seconds: [usize; 2], // Zweiter und vorletzter Punkt
    pub pts: Vec<usize>, // Alle Punkte der Linie
}
impl Line {
    // Erstellt eine neue Linie
    fn new(pts: Vec<usize>) -> Self {
        let ends = [pts[0], pts[pts.len() - 1]];
        let seconds = [pts[1], pts[pts.len() - 2]];
        Self { ends, pts, seconds }
    }
    // Gibt die Länge der Linie zurück
    pub fn get_cost(&self, costs: &CostList) -> f64 {
        let mut cost = 0.0;
        for i in 0..self.pts.len() - 1 {
            cost += costs.get(self.pts[i], self.pts[i + 1]).unwrap();
        }
        cost
    }
}
/// Findet alle Linien in den Punkten
pub fn find_lines(costs: &CostList, angle_list: &AngleOkList, line_min: usize) -> Vec<Line> {
    println!("Linien finden...");
    let n_pts = costs.size;
    // Erstellen einer HashMap, um schnell auf mögliche Liniensegmente zugreifen zu können
    let mut line_segments = HashMap::new();

    // Finden von Liniensegmenten durch Iterieren über alle Punkte
    for start in 0..n_pts {
        // Bestimmen der drei nächsten Punkte und ihrer Abstände
        let mut nexts = (0..n_pts)
            .filter_map(|b| Some((b, (*costs.get(start, b))?)))
            .collect::<Vec<_>>();
        nexts.sort_by(|a, b| a.1.total_cmp(&b.1));
        let (pt_0, dist_0) = nexts[0];
        let (pt_1, dist_1) = nexts[1];
        let (_, dist_2) = nexts[2];

        // Überprüfen, ob die nächsten drei Punkte ein Liniensegment bilden
        let security = 1.5;
        if dist_1 < dist_0 * security
            && dist_2 > dist_1 * security
            && angle_list.is_ok(pt_0, start, pt_1)
        {
            // Speichern des möglichen Liniensegments in der HashMap
            line_segments.insert(start, (pt_0, pt_1));
        }
    }

    // Zusammenfügen von Liniensegmenten zu Linien
    let mut lines = vec![];
    let mut used_pts = vec![false; n_pts];
    while !line_segments.is_empty() {
        // Erstellen einer neuen Linie
        let mut line = LinkedList::new();

        // Auswahl eines zufälligen Liniensegments als Startpunkt
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

        // Erweiterung der Linie in eine Richtung
        while let Some((a, b)) = line_segments.remove(&end) {
            //Finde den Punkt, der noch nicht Teil der Linie ist
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

        // Erweiterung der Linie in die andere Richtung
        while let Some((a, b)) = line_segments.remove(&start) {
            //Finde den Punkt, der noch nicht Teil der Linie ist
            let new_start = if a == p_pt { b } else { a };
            if used_pts[new_start] {
                break;
            }
            line.push_front(new_start);
            used_pts[new_start] = true;
            p_pt = start;
            start = new_start;
        }

        // Überprüfen, ob die Linie ausreichend lang ist und in eine Linie umgewandelt werden kann
        let line = line.into_iter().collect::<Vec<_>>();
        if line.len() >= line_min {
            lines.push(Line::new(line));
        }
    }
    lines
}
/// Speichert Sprünge
#[derive(Clone)]
pub struct Skip {
    pub end: usize,            //Ende der Linie
    pub second_pt: usize,      //Der Punkt nach dem Anfang der Linie
    pub penultimate_pt: usize, //Der Punkt vor dem Ende der Linie
    pub cost: f64,             //Wie lange die Linie ist
}
impl Skip {
    // Erstellt einen neuen Sprung
    fn new(end: usize, second_pt: usize, penultimate_pt: usize, cost: f64) -> Self {
        Self {
            end,
            second_pt,
            penultimate_pt,
            cost,
        }
    }
}
/// Erstellt eine Liste von Sprüngen aus einer Liste von Linien
pub fn get_skips(costs: &CostList, lines: &Vec<Line>) -> Vec<Option<Skip>> {
    //Gibt an, ob der Punkt an einem bestimmten Index Endpunkt einer Linie ist
    let mut skips = vec![None; costs.size]; 
    for line in lines {
        // Ein Sprung für beide Enden der Linie erstellen
        for (skip_start, skip_end) in [(0, 1), (1, 0)].iter() {
            let second_pt = line.seconds[*skip_start];
            let penultimate_pt = line.seconds[*skip_end];
            let skip = Skip::new(
                line.ends[*skip_end],
                second_pt,
                penultimate_pt,
                line.get_cost(costs),
            );
            // Füge den Sprung in die Liste ein
            skips[line.ends[*skip_start]] = Some(skip);
        }
    }
    skips
}
//fügt die linien wieder in die liste ein
pub fn insert_lines(path: &Vec<usize>, lines: Vec<Line>) -> Vec<usize> {
    // Die Linie aus der sicht des Endpunktes aus,
    // gespeichert nach Endpunkt
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
    for pt in path {
        // Wenn der Punkt Teil einer Linie ist, füge die Linie ein
        if let Some(line) = line_map.remove(&pt) {
            // Es wird überprüft, ob die Linie gerade schon eingefügt wurde
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