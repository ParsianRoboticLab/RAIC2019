#[derive(Copy, Clone, Debug, Default)]
struct Line2 {
    a: f64,
    b: f64,
    c: f64,
}


impl Line2 {
    fn new(a: f64, b: f64, c:f64) -> Self {
        Self {a,b,c}
    }

    fn new2(_vec1: Vec2, _vec2: Vec2) -> Self {
        Self{
            ..Default::default()
        }.assign(_vec1, _vec2)
    }

    fn assign(&mut self, _vec1: Vec2, _vec2: Vec2) -> Self {
            self.a = -(_vec2.y - _vec1.y);
            self.b =  _vec2.x - _vec1.x;
            self.c =  (-self.a * (_vec1.x)) - (self.b * (_vec1.y));
            *self
    }

    fn a(&self) -> f64 {
        self.a
    }

    fn b(&self) -> f64 {
        self.b
    }

    fn c(&self) -> f64 {
        self.c
    }

    fn getX(&self, y: f64) -> f64 {
        - (self.b * y + self.c) / self.a
    }

    fn getY(&self, x: f64) -> f64 {
        - (self.a * x + self.c) / self.b
    }

    fn dist(&self, v : Vec2) -> f64 {
        ((self.a * v.x + self.b * v.y + self.c)/(self.a * self.a + self.b * self.b).sqrt()).abs()
    }

    fn dist2(&self, v : Vec2) -> f64 {
        let d = self.a * v.x + self.b * v.y + self.c;
        (d * d)/(self.a * self.a + self.b * self.b)
    }

    fn isParallel(&self, l: Line2) -> bool {
        (self.a * l.b() - l.a() - self.b).abs() < EPSILON
    }

    fn intersection(&self, other : Line2) -> Vec2 {
        Line2::rintersection(*self, other)
    }
}

trait Intersection {
    fn rintersection(l1 : Line2, l2 : Line2) -> Vec2 ;
    fn perpendicular_bisector(v1: Vec2, v2: Vec2) -> Line2;
}

impl Intersection for Line2 {
    fn rintersection(l1 : Line2, l2 : Line2) -> Vec2 {
        let t = l1.a() * l2.b() - l1.b() * l2.a();
        if t.abs() < EPSILON {
            return VEC2INVALID
        }
        Vec2{
            x: (l1.b() * l2.c() - l2.b() * l1.c())/t,
            y: (l2.a() * l1.c() - l1.a() * l2.c())/t
        }
    }

    fn perpendicular_bisector(v1: Vec2, v2: Vec2) -> Line2 {
        if (v2.x - v1.x).abs() < EPSILON && (v2.y - v1.y) < EPSILON {
            return Line2::new2(v1, Vec2{x:v1.x + 1.0, y: v1.y});
        }
        let t = (v2.x*v2.x - v1.x*v1.x + v2.y*v2.y - v1.y*v1.y) * -0.5;
        Line2{
            a: v2.x - v1.x,
            b: v2.y - v1.y,
            c: t,
        }
    }


}
