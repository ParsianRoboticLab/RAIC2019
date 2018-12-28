// Common interface for ball and robots
trait Entity {
    // Position of the entity in XZ plane
    fn position(&self) -> Vec2;
    // Velocity of the entity in XZ plane
    fn velocity(&self) -> Vec2;
    // Height
    fn height(&self) -> f64;
}

impl Entity for Robot {
    fn position(&self) -> Vec2 {
        Vec2::new(self.x, self.z)
    }
    fn velocity(&self) -> Vec2 {
        Vec2::new(self.velocity_x, self.velocity_z)
    }
    fn height(&self) -> f64 {
        self.y
    }
}

impl Entity for Ball {
    fn position(&self) -> Vec2 {
        Vec2::new(self.x, self.z)
    }
    fn velocity(&self) -> Vec2 {
        Vec2::new(self.velocity_x, self.velocity_z)
    }
    fn height(&self) -> f64 {
        self.y
    }
}