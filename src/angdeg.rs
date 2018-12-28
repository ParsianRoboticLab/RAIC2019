use std::f64::consts::PI;

const DEG2RAD : f64 = PI/180.0;
const RAD2DEG : f64 = 180.0/PI;

#[derive(Copy, Clone, Debug, Default)]
struct AngDeg {
    degree: f64,
}

impl AngDeg {
    fn new(deg:f64) -> Self {
        Self {
            degree: deg,
        }.normalize()
    }

    fn normalize(&mut self) -> Self {
        if self.degree < -360.0 || 360.0 < self.degree {
            self.degree %= 360.0;
        }
        if self.degree < -180.0 {
            self.degree += 360.0;
        }
        if self.degree > 180.0 {
            self.degree -= 360.0;
        }
        *self
    }

    fn deg(&self) -> f64 {
        self.degree
    }

    fn abs(&self) -> f64 {
        self.degree.abs()
    }

    fn radian(&self) -> f64 {
        self.degree * DEG2RAD
    }
}

impl std::ops::Add for AngDeg {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            degree: self.degree + other.deg()
        }.normalize()
    }
}

impl std::ops::AddAssign for AngDeg {
    fn add_assign(&mut self, other: Self) {
        self.degree += other.deg();
        self.normalize();
    }
}

impl std::ops::Sub for AngDeg {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            degree: self.degree - other.deg()
        }.normalize()
    }
}

impl std::ops::SubAssign for AngDeg {
    fn sub_assign(&mut self, other: Self) {
        self.degree -= other.deg();
        self.normalize();
    }
}

impl std::ops::Mul for AngDeg {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        Self {
            degree: self.degree * other.deg()
        }.normalize()
    }
}

impl std::ops::MulAssign for AngDeg {
    fn mul_assign(&mut self, other: Self) {
        self.degree *= other.deg();
        self.normalize();
    }
}

impl std::ops::Div for AngDeg {
    type Output = Self;
    fn div(self, other: Self) -> Self {
        Self {
            degree: self.degree / other.deg()
        }.normalize()
    }
}

impl std::ops::DivAssign for AngDeg {
    fn div_assign(&mut self, other: Self) {
        self.degree /= other.deg();
        self.normalize();
    }
}

impl std::ops::Neg for AngDeg {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            degree: -self.degree
        }.normalize()
    }
}
