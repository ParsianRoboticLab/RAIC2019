trait Simulation {
    fn collide_entities(_a: &mut Entity3, _b: &mut Entity3);
    fn collide_with_arena(_e: &mut Entity3) -> Vec3;
    fn move_e(_e: &mut Entity3, delta_time: f64);
}

fn clamp(_v : &Vec3, max: f64) -> Vec3 {
    if _v.len() > max {
        return _v.normalize() * max
    }
    *_v
}

fn dan_to_arena(_v : &Vec3) -> (f64, Vec3) {
    (0.0, VEC3INVALID)
}

impl Simulation for MyStrategy {
    fn collide_entities(_a: &mut Entity3, _b: &mut Entity3) {
        let delta_position = _b.position3() - _a.position3();
        let distance = delta_position.len();
        let penetration = _a.radius() + _b.radius() - distance;
        if penetration > 0.0 {
            let a_pos = _a.position3();
            let b_pos = _b.position3();
            let a_vel = _a.velocity3();
            let b_vel = _b.velocity3();
            let k_a = (1.0 / _a.mass()) / ((1.0 / _a.mass()) + (1.0  / _b.mass()));
            let k_b = (1.0 / _b.mass()) / ((1.0 / _a.mass()) + (1.0  / _b.mass()));
            let normal = delta_position.normalize();
            _a.set_position(&(a_pos - (normal * penetration * k_a)));
            _b.set_position(&(b_pos + (normal * penetration * k_b)));
            let delta_vel = (_b.velocity3() - _a.velocity3()).inner_product(&normal);
            //+ _b.radius_change_speed() - _a.radius_change_speed();
            if delta_vel < 0.0 {
                let impulse = normal * (1.0 + 4.5) * delta_vel;
                _a.set_velocity(&(a_vel + (impulse * k_a)));
                _b.set_velocity(&(b_vel - (impulse * k_b)));
            }
        }
    }

    fn collide_with_arena(_e: &mut Entity3) -> Vec3 {
        let (distance, normal) = dan_to_arena(&_e.position3());
        let penetration = _e.radius() - distance;
        if penetration > 0.0 {
            let pos = _e.position3();
            let vel = _e.velocity3();
            let arena_e = _e.arena_e();
            _e.set_position(&(pos + (normal * penetration)));
            let velocity = vel.inner_product(&normal); // - _e.radius_change_speed();
            if velocity < 0.0 {
                _e.set_velocity(&(vel - (normal * (1.0 + arena_e) * velocity)));
                return normal;
            }
        }
        VEC3INVALID
    }


    fn move_e(_e: &mut Entity3, delta_time: f64) {
        let gravity = 30.0;
        let pos = _e.position3();
        let vel = _e.velocity3();
        let h = _e.height3();
        let ms = _e.max_speed();
        _e.set_velocity(&clamp(&vel, ms));
        _e.set_position(&(pos + (vel * delta_time)));
        _e.set_height(h - (gravity * delta_time * delta_time / 2.0));
        _e.set_height(h - (gravity * delta_time));
    }


}
