// We will need to work with 2d vertors
#[derive(Copy, Clone, Debug, Default)]
struct Vec2 {
    x: f64,
    y: f64,
}

impl Vec2 {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y }
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
        (*self - other).len()
    }

    fn th(self) -> AngDeg {
        (self.normalize().y).atan2(self.normalize().x)
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
