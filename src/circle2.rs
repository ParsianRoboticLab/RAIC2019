#[derive(Copy, Clone, Debug, Default)]
struct Circle2 {
    center: Vec2,
    radius: f64,
}

fn SQUARE(v : &f64) -> f64{
    v * v
}

fn QUADRATIC_FOMULA(a: &f64, b: &f64, c:&f64, sol1: &mut f64, sol2: &mut f64) -> usize {
    let mut d = SQUARE(b) - 4.0 * a * c;
    if d.abs() < std::f64::EPSILON {
        *sol1 = -b / (2.0 * a);
        1
    } else if d < 0.0 {
        0
    } else {
        d = d.sqrt();
        *sol1 = (-b + d) / (2.0 * a);
        *sol2 = (-b - d) / (2.0 * a);
        2
    }
}

impl Circle2 {
    fn new(center : Vec2, radius: f64) -> Self{
        Circle2 {center, radius}
    }

    fn area(&self) -> f64 {
        std::f64::consts::PI * self.radius * self.radius
    }

    fn contains(&self, v: &Vec2) -> bool {
        self.center.dist2(*v) < self.radius * self.radius
    }

    fn intersection_line(&self, l : &Line2) -> (usize, Vec2, Vec2) {
        let mut sol1 = VEC2INVALID;
        let mut sol2 = VEC2INVALID;
        if l.a.abs() < std::f64::EPSILON {
            if l.b.abs() < std::f64::EPSILON {
                return (0, VEC2INVALID, VEC2INVALID);
            }
            let mut x1 = INVALID_;
            let mut x2 = INVALID_;
            let n_sol = QUADRATIC_FOMULA(&1.0, &(-2.0 * self.center.x),
                            &((SQUARE(&self.center.x))
                            + (SQUARE(&(l.c / l.b + self.center.y)))
                            - (SQUARE(&self.radius))),
                            &mut x1, &mut x2);
            if n_sol > 0 {
                let y1 = -l.c / l.b;
                sol1.assign(x1, y1);
                if n_sol > 1 {
                    sol2.assign(x2, y1);
                }
            }
            (n_sol, sol1, sol2)
        } else {
            let m = l.b / l.a;
            let d = l.c / l.a;
            let a = 1.0 * m * m;
            let b = 2.0 * (-self.center.y + (d + self.center.x) * m);
            let c = SQUARE(&(d + self.center.x)) + SQUARE(&self.center.y) - SQUARE(&self.radius);
            let mut y1 : f64 = 0.0;
            let mut y2 : f64 = 0.0;
            let n_sol = QUADRATIC_FOMULA(&a, &b, &c, &mut y1, &mut y2);
            if n_sol > 0 {
                sol1.assign(l.getX(y1), y1);
            }
            if n_sol > 1 {
                sol2.assign(l.getX(y2), y2);
            }
            (n_sol, sol1, sol2)
        }

    }


}
