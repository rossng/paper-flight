//! Problem file written by `reference/export.ts`: the mesh, the material model and the
//! launch, exactly as the game sets them up. Plain text so the solver needs no parser crate.
use crate::vec::V3;
use std::collections::HashMap;

pub struct Hinge { pub ids: [usize; 4], pub rest: f64, pub stiffness: f64, pub crease: bool }
pub struct Edge { pub a: usize, pub b: usize, pub rest: f64 }
/// Known stacking side of a vertex relative to a material panel (`owner`), as a sign along
/// that panel's face normals.
pub struct Side { pub vertex: usize, pub owner: usize, pub sign: f64 }

pub struct Problem {
    pub scalars: HashMap<String, f64>,
    pub rest: Vec<V3>, pub start: Vec<V3>, pub masses: Vec<f64>, pub uv: Vec<[f64; 2]>,
    pub faces: Vec<[usize; 3]>, pub owners: Vec<usize>, pub has_owners: bool,
    pub hinges: Vec<Hinge>, pub edges: Vec<Edge>, pub sides: Vec<Side>,
    pub nose: usize, pub tail: usize, pub left: usize, pub right: usize,
}

impl Problem {
    pub fn scalar(&self, name: &str) -> f64 {
        *self.scalars.get(name).unwrap_or_else(|| panic!("problem file lacks scalar '{name}'"))
    }
    pub fn scalar_or(&self, name: &str, default: f64) -> f64 { *self.scalars.get(name).unwrap_or(&default) }

    pub fn parse(text: &str) -> Result<Problem, String> {
        let mut lines = text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#'));
        let header = lines.next().ok_or("empty problem file")?;
        if header != "paperflight-reference-problem 1" { return Err(format!("unsupported problem header: {header}")); }
        let mut p = Problem {
            scalars: HashMap::new(), rest: vec![], start: vec![], masses: vec![], uv: vec![], faces: vec![], owners: vec![],
            has_owners: false, hinges: vec![], edges: vec![], sides: vec![], nose: 0, tail: 0, left: 0, right: 0,
        };
        let numbers = |line: &str| -> Result<Vec<f64>, String> {
            line.split_whitespace().map(|t| t.parse::<f64>().map_err(|e| format!("bad number '{t}': {e}"))).collect()
        };
        while let Some(line) = lines.next() {
            let mut words = line.split_whitespace();
            let key = words.next().unwrap();
            let rest: Vec<&str> = words.collect();
            let count = || -> Result<usize, String> { rest.first().ok_or("missing count")?.parse::<usize>().map_err(|e| e.to_string()) };
            let mut rows = |n: usize, width: usize| -> Result<Vec<Vec<f64>>, String> {
                (0..n).map(|_| {
                    let row = numbers(lines.next().ok_or("truncated problem file")?)?;
                    if row.len() != width { return Err(format!("expected {width} values in a '{key}' row, found {}", row.len())); }
                    Ok(row)
                }).collect()
            };
            match key {
                "scalar" => { p.scalars.insert(rest[0].to_string(), rest[1].parse::<f64>().map_err(|e| e.to_string())?); }
                "landmarks" => {
                    let v: Vec<usize> = rest.iter().map(|t| t.parse::<usize>().unwrap()).collect();
                    p.nose = v[0]; p.tail = v[1]; p.left = v[2]; p.right = v[3];
                }
                "vertices" => for r in rows(count()?, 9)? {
                    p.rest.push([r[0], r[1], r[2]]); p.start.push([r[3], r[4], r[5]]); p.masses.push(r[6]); p.uv.push([r[7], r[8]]);
                },
                "faces" => for r in rows(count()?, 4)? {
                    p.faces.push([r[0] as usize, r[1] as usize, r[2] as usize]);
                    if r[3] >= 0.0 { p.has_owners = true; }
                    p.owners.push(if r[3] >= 0.0 { r[3] as usize } else { usize::MAX });
                },
                "hinges" => for r in rows(count()?, 7)? {
                    p.hinges.push(Hinge { ids: [r[0] as usize, r[1] as usize, r[2] as usize, r[3] as usize], rest: r[4], stiffness: r[5], crease: r[6] != 0.0 });
                },
                "edges" => for r in rows(count()?, 3)? { p.edges.push(Edge { a: r[0] as usize, b: r[1] as usize, rest: r[2] }); },
                "sides" => for r in rows(count()?, 3)? { p.sides.push(Side { vertex: r[0] as usize, owner: r[1] as usize, sign: r[2] }); },
                "end" => break,
                other => return Err(format!("unknown section '{other}'")),
            }
        }
        if p.rest.is_empty() || p.faces.is_empty() { return Err("problem has no mesh".into()); }
        if !p.has_owners { for (i, o) in p.owners.iter_mut().enumerate() { *o = i; } }
        Ok(p)
    }
}
