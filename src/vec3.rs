// We will need to work with 3d vertors
#[derive(Copy, Clone, Debug, Default)]
struct Vec3 {
    x: f64,
    y: f64,
    h: f64
}

const VEC3INVALID : Vec3 = Vec3{x:INVALID_,y:INVALID_,h:INVALID_};


impl Vec3 {
    fn new(x: f64, y: f64, h:f64) -> Self {
    Self { x, y, h }
    }
    // Finding length of the vector
    fn len(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.h * self.h).sqrt()
    }
    // Normalizing vector (setting its length to 1)
    fn normalize(self) -> Self {
        self * (1.0 / self.len())
    }

    fn dist(&self, other: Self) -> f64 {
        (*self - other).len()
    }

    fn toVec2(&self) -> Vec2 {
        Vec2{x:self.x,y:self.y}
    }

    fn is_valid(&self) -> bool {
        (self.x - INVALID_).abs() > std::f64::EPSILON &&
        (self.y - INVALID_).abs() > std::f64::EPSILON &&
        (self.h - INVALID_).abs() > std::f64::EPSILON
    }


    fn inner_product(&self, p: &Vec3) -> f64 {
        self.x * p.x + self.y * p.y + self.h * p.h
    }
}

// Subtraction operation for vectors
impl std::ops::Sub for Vec3 {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self::new(self.x - b.x, self.y - b.y, self.h - b.h)
    }
}

// Subtraction operation for vectors
impl std::ops::SubAssign for Vec3 {
    fn sub_assign(&mut self, b: Self) {
        *self = Self::new(self.x - b.x, self.y - b.y, self.h - b.h)
    }
}

// Addition for vectors
impl std::ops::Add for Vec3 {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self::new(self.x + b.x, self.y + b.y, self.h + b.h)
    }
}

// Addition for vectors
impl std::ops::AddAssign for Vec3 {
    fn add_assign(&mut self, b: Self) {
        *self = Self::new(self.x + b.x, self.y + b.y, self.h + b.h);
    }
}

// Multiplying vector by a number
impl std::ops::Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, k: f64) -> Self {
        Self::new(self.x * k, self.y * k, self.h * k)
    }
}
