// Common interface for ball and robots
trait Entity3 {
    // Position of the entity in XZ plane
    fn position(&self) -> Vec3;
    // Velocity of the entity in XZ plane
    fn velocity(&self) -> Vec3;
    // Radius of object
    fn radius(&self) -> f64;
    // mass
    fn mass() -> f64;

    fn set_position(&mut self, p : &Vec3);
    fn set_velocity(&mut self, v : &Vec3);
    fn max_speed() -> f64;
    fn arena_e() -> f64;
    fn set_height(&mut self, h: f64);
    fn height(&self);
}

impl Entity3 for Robot {
    fn position(&self) -> Vec2 {
        Vec3::new(self.x, self.z, self.y)
    }
    fn velocity(&self) -> Vec2 {
        Vec2::new(self.velocity_x, self.velocity_z, self.velocity_y)
    }
    fn radius(&self) -> f64 {
        self.radius
    }

    fn mass() -> f64 {
        2.0
    }
    fn set_position(&mut self) {
        self.x = p.x;
        self.y = p.y;
        self.z = p.z;
    }
    fn set_velocity(&mut self, v : &Vec3) {
        self.velocity_x = v.x;
        self.velocity_y = v.y;
        self.velocity_z = v.z;
    }
    fn height(&self) {
        self.y
    }
    fn set_height(&mut self, h: f64) {
        self.y = h;
    }

    fn max_speed() {
        100.0
    }
    fn arena_e() {
        0.0
    }

}

impl Entity3 for Ball {
    fn position(&self) -> Vec2 {
        Vec3::new(self.x, self.z, self.y)
    }
    fn velocity(&self) -> Vec2 {
        Vec2::new(self.velocity_x, self.velocity_z, self.velocity_y)
    }
    fn radius(&self) -> f64 {
        self.radius
    }

    fn mass(&self) -> f64 {
        1.0
    }
    fn set_position(&mut self) {
        self.x = p.x;
        self.y = p.y;
        self.z = p.z;
    }
    fn set_velocity(&mut self, v : &Vec3) {
        self.velocity_x = v.x;
        self.velocity_y = v.y;
        self.velocity_z = v.z;
    }
    fn arena_e() {
        0.7
    }
    fn set_height(&mut self, h: f64) {
        self.y = h;
    }
    fn height(&self) {
        self.y
    }
}
