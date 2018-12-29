// Common interface for ball and robots
trait Entity {
    // Position of the entity in XZ plane
    fn position(&self) -> Vec2;
    // Velocity of the entity in XZ plane
    fn velocity(&self) -> Vec2;
    // Height
    fn height(&self) -> f64;
    // Height Velocity
    fn hVel(&self) -> f64;
    // Radius of object
    fn radius(&self) -> f64;

    fn mass(&self) -> f64;
}

impl Entity for Robot {
    fn position(&self) -> Vec2 {
        Vec2::new(self.x, self.z)
    }
    fn velocity(&self) -> Vec2 {
        Vec2::new(self.velocity_x, self.velocity_z)
    }
    fn hVel(&self) -> f64 {
        self.velocity_y
    }
    fn height(&self) -> f64 {
        self.y
    }
    fn radius(&self) -> f64 {
        self.radius
    }

    fn mass(&self) -> f64 {
        2.0
    }

}

impl Entity for Ball {
    fn position(&self) -> Vec2 {
        Vec2::new(self.x, self.z)
    }
    fn velocity(&self) -> Vec2 {
        Vec2::new(self.velocity_x, self.velocity_z)
    }
    fn hVel(&self) -> f64 {
        self.velocity_y
    }
    fn height(&self) -> f64 {
        self.y
    }

    fn radius(&self) -> f64 {
        self.radius
    }

    fn mass(&self) -> f64 {
        1.0
    }
}
