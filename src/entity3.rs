// Common interface for ball and robots
trait Entity3 {
    // Position of the entity in XZ plane
    fn position3(&self) -> Vec3;
    // Velocity of the entity in XZ plane
    fn velocity3(&self) -> Vec3;
    // Touch normal
    fn touch_noraml(&self) -> Vec3;
    // Radius of object
    fn radius(&self) -> f64;
    // mass
    fn mass(&self) -> f64;

    fn set_position(&mut self, p : &Vec3);
    fn set_velocity(&mut self, v : &Vec3);
    fn set_height(&mut self, h: f64);

    fn max_speed(&self) -> f64;
    fn arena_e(&self) -> f64;
    fn height3(&self) -> f64;
}

impl Entity3 for Robot {
    fn position3(&self) -> Vec3 {
        Vec3::new(self.x, self.z, self.y)
    }
    fn velocity3(&self) -> Vec3 {
        Vec3::new(self.velocity_x, self.velocity_z, self.velocity_y)
    }
    fn radius(&self) -> f64 {
        self.radius
    }

    fn mass(&self) -> f64 {
        2.0
    }
    fn set_position(&mut self, p: &Vec3) {
        self.x = p.x;
        self.y = p.h;
        self.z = p.y;
    }
    fn set_velocity(&mut self, v : &Vec3) {
        self.velocity_x = v.x;
        self.velocity_y = v.h;
        self.velocity_z = v.y;
    }
    fn height3(&self) -> f64{
        self.y
    }
    fn set_height(&mut self, h: f64) {
        self.y = h;
    }

    fn max_speed(&self) -> f64{
        100.0
    }
    fn arena_e(&self) -> f64 {
        0.0
    }

    fn touch_noraml(&self) -> Vec3 {
        Vec3 {
            x: self.touch_normal_x.unwrap(),
            y: self.touch_normal_z.unwrap(),
            h: self.touch_normal_y.unwrap(),
        }
    }

}

impl Entity3 for Ball {
    fn position3(&self) -> Vec3 {
        Vec3::new(self.x, self.z, self.y)
    }
    fn velocity3(&self) -> Vec3 {
        Vec3::new(self.velocity_x, self.velocity_z, self.velocity_y)
    }
    fn radius(&self) -> f64 {
        self.radius
    }

    fn mass(&self) -> f64 {
        1.0
    }
    fn set_position(&mut self, p: &Vec3) {
        self.x = p.x;
        self.y = p.h;
        self.z = p.y;
    }
    fn max_speed(&self) -> f64{
        100.0
    }
    fn set_velocity(&mut self, v : &Vec3) {
        self.velocity_x = v.x;
        self.velocity_y = v.h;
        self.velocity_z = v.y;
    }
    fn arena_e(&self) -> f64{
        0.7
    }
    fn set_height(&mut self, h: f64) {
        self.y = h;
    }
    fn height3(&self) -> f64{
        self.y
    }
    fn touch_noraml(&self) -> Vec3 {
        VEC3INVALID
    }
}
