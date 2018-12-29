struct Simulation {

}

impl Simulation {
    fn clamp(_v : &Vec3, max: f64) -> Vec3 {
        if _v.len() > max {

            return _v.normalize() * max
        }
        *_v
    }

    fn dan_to_arena(_v : &Vec3) -> (f64, Vec3) {
        (0.0, VEC3INVALID)
    }

    fn collide_entities(_a: &mut Entity3, _b: &mut Entity3, _radius_change_speed: f64, _rules: &Rules) {
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
            let delta_vel = (_b.velocity3() - _a.velocity3()).inner_product(&normal) - _radius_change_speed;
            //+ _b.radius_change_speed() - _a.radius_change_speed();
            if delta_vel < 0.0 {
                let impulse = normal * (1.0 + (_rules.MIN_HIT_E + _rules.MAX_HIT_E)/2.0) * delta_vel;
                _a.set_velocity(&(a_vel + (impulse * k_a)));
                _b.set_velocity(&(b_vel - (impulse * k_b)));
            }
        }
    }

    fn collide_with_arena(_e: &mut Entity3, _radius_change_speed: f64) -> Vec3 {
        let (distance, normal) = Self::dan_to_arena(&_e.position3());
        let penetration = _e.radius() - distance;
        if penetration > 0.0 {
            let pos = _e.position3();
            let vel = _e.velocity3();
            let arena_e = _e.arena_e();
            _e.set_position(&(pos + (normal * penetration)));
            let velocity = vel.inner_product(&normal) - _radius_change_speed;
            if velocity < 0.0 {
                _e.set_velocity(&(vel - (normal * (1.0 + arena_e) * velocity)));
                return normal;
            }
        }
        VEC3INVALID
    }


    fn move_e(_e: &mut Entity3, delta_time: f64, _rules: &Rules) {
        let pos = _e.position3();
        let vel = _e.velocity3();
        let h = _e.height3();
        let ms = _e.max_speed();
        _e.set_velocity(&Self::clamp(&vel, ms));
        _e.set_position(&(pos + (vel * delta_time)));
        _e.set_height(h - (_rules.GRAVITY * delta_time * delta_time / 2.0));
        _e.set_height(h - (_rules.GRAVITY * delta_time));
    }

    fn update(_me : &mut Robot, _ball: &mut Ball, _action: &Action, _rules: &Rules, delta_time: f64) {
            if _me.touch {
                let mut target_vel = Self::clamp(&_action.target_vel(), _rules.ROBOT_MAX_GROUND_SPEED);
                target_vel -= _me.touch_normal() * _me.touch_normal().inner_product(&target_vel);
                let target_vel_change = target_vel - _me.velocity3();
                if target_vel_change.len() > 0.0 {
                    let acc = _rules.ROBOT_ACCELERATION * _me.touch_normal_y.unwrap().max(0.0);
                    let robot_vel = _me.velocity3();
                    _me.set_velocity(&(robot_vel + Self::clamp(&(_action.target_vel() * acc * delta_time), target_vel_change.len())));
                }
                // TODO : USE NITRO
                Self::move_e(_me, delta_time, _rules);
                _me.radius = _rules.ROBOT_MIN_RADIUS + (_rules.ROBOT_MAX_RADIUS - _rules.ROBOT_MIN_RADIUS) * _action.jump_speed / _rules.ROBOT_MAX_JUMP_SPEED;
            }
            Self::move_e(_ball, delta_time, _rules);
            Self::collide_entities(_me, _ball, _action.jump_speed, _rules);
            let collision_normal = Self::collide_with_arena(_me, _action.jump_speed);
            if ! collision_normal.is_valid() {
                _me.touch = false;
            } else {
                _me.touch = true;
                _me.set_touch_normal(&collision_normal);
            }
            Self::collide_with_arena(_ball, 0.0);
    }

    fn tick(_me : &mut Robot, _ball: &mut Ball, _action: &Action, _rule: &Rules, _time: f64) {
        let delta_time = _time / _rule.TICKS_PER_SECOND as f64;
        for _ in 0 .. _rule.MICROTICKS_PER_TICK - 1 {
            Self::update(_me, _ball, _action, _rule, delta_time / _rule.MICROTICKS_PER_TICK as f64);
        }
    }
}
