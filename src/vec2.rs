// We will need to work with 2d vertors
#[derive(Copy, Clone, Debug, Default)]
struct Vec2 {
    x: f64,
    y: f64,
}

const INVALID_ : f64 = 5000.0;
const VEC2INVALID : Vec2 = Vec2{x:INVALID_,y:INVALID_};

fn angle_of(a: &Vec2, o: &Vec2, b: &Vec2) -> AngDeg {
    let a1 = Vec2::new(a.x - o.x, a.y - o.y);
    let a2 = Vec2::new(b.x - o.x, b.y - o.y);
    AngDeg {
        degree: a1.th().deg() - a2.th().deg()
    }
}

impl Vec2 {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    fn assign(&mut self, x: f64, y:f64) {
        self.x = x;
        self.y = y;
    }

    fn from_polar(mag: f64, theta: AngDeg) -> Self{
        Self {
            x: mag * theta.deg().cos(),
            y: mag * theta.deg().sin(),
        }
    }
    // Finding length of the vector
    fn len(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    // Normalizing vector (setting its length to 1)
    fn normalize(self) -> Self {
        self * (1.0 / self.len())
    }

    fn dist(&self, other: Self) -> f64 {
        self.dist2(other).sqrt()
    }

    fn dist2(&self, other: Self) -> f64 {
        (self.x - other.x) * (self.x - other.x) + (self.y - other.y) * (self.y - other.y)
    }

    fn th(&self) -> AngDeg {
        AngDeg{degree:(self.normalize().y).atan2(self.normalize().x) * RAD2DEG}
    }

    fn is_valid(&self) -> bool {
        (self.x - INVALID_).abs() > std::f64::EPSILON &&
        (self.y - INVALID_).abs() > std::f64::EPSILON
    }

    fn rotate(&mut self, deg: f64) -> Vec2 {
        let c = (deg * DEG2RAD).cos();
        let s = (deg * DEG2RAD).sin();
        self.x = self.x * c - self.y * s;
        self.y = self.x * s + self.y * c;
        *self
    }

    fn rotateVector(&self, deg: f64) -> Vec2 {
        let mut v = self.clone();
        v.rotate(deg);
        v
    }

    fn inner_product(&self, p: &Vec2) -> f64 {
        self.x * p.x + self.y * p.y
    }
}

// Subtraction operation for vectors
impl std::ops::Sub for Vec2 {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self::new(self.x - b.x, self.y - b.y)
    }
}

// Addition for vectors
impl std::ops::Add for Vec2 {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self::new(self.x + b.x, self.y + b.y)
    }
}

// Multiplying vector by a number
impl std::ops::Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, k: f64) -> Self {
        Self::new(self.x * k, self.y * k)
    }
}
