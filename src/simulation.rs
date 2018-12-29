use rand::prelude::*;

trait Simulation {
    fn collide_entities(_a: &Entity3, _b: &Entity3);
    fn collide_with_arena(_e: &Entity3);
    fn move_e(_e: &Entity3, delta_time: f64);
}

impl Simulation for MyStrategy {
    fn collide_entities(_a: &mut Entity3, _b: &mut Entity3) {
        let delta_position = _b.position() - _a.position();
        let distance = delta_position.len();
        let penetration = _a.radius() + _b.radius() - distance;
        if penetration > 0.0 {
            let k_a = (1.0 / _a.mass()) / ((1.0 / _a.mass()) + (1.0  / _b.mass()));
            let k_b = (1.0 / _b.mass()) / ((1.0 / _a.mass()) + (1.0  / _b.mass()));
            let normal = delta_position.normalize();
            _a.set_position(_a.position() - (normal * penetration * k_a));
            _b.set_position(_b.position() + (normal * penetration * k_b));
            let delta_vel = (_b.velocity() - _a.velocity()).inner_product(normal);
            //+ _b.radius_change_speed() - _a.radius_change_speed();
            if delta_vel < 0 {
                let impulse = (1.0 + 4.5) * delta_vel * normal;
                _a.set_velocity(_a.velocity() + (impulse * k_a));
                _b.set_velocity(_b.velocity() - (impulse * k_b));
            }
        }
    }

    fn collide_with_arena(_e: &mut Entity3) -> Vec3 {
        let (distance, noraml) = self.dan_to_arena(_e.position());
        let penetration = _e.radius() - distance;
        if penetration > 0.0 {
            _e.set_position(_e.position() + (penetration * noraml));
            let velocity = _e.velocity().inner_product(normal); // - _e.radius_change_speed();
            if velocity < 0.0 {
                _e.set_velocity(_e.velocity() - ((1 + _e.arena_e()) * velocity * normal));
                return normal;
            }
        }
        VEC3INVALID
    }

    fn clamp(_v : &Vec3, max: f64) -> Vec3 {
        if _v.len() > max {
            return _v.normalize() * max
        }
        _v
    }

    fn move_e(_e: &Entity3, delta_time: f64) {
        let gravity = 30.0;
        _e.set_velocity(clamp(_e.velocity(), _e.max_speed()));
        _e.set_position(_e.position() + (_e.velocity() * delta_time));
        _e.set_height(_e.height() - (gravity * delta_time * delta_time / 2.0));
        _e.set_height(_e.height() - (gravity * delta_time));
    }


}
