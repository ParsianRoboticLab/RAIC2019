#[derive(Copy, Clone, Debug, Default)]
struct Simulation {
}

impl Simulation {
    fn clamp(_v : &Vec3, max: f64) -> Vec3 {
        if _v.len() > max {

            return _v.normalize() * max
        }
        *_v
    }

    fn collide_entities_col(_a: &mut Entity3, _b: &mut Entity3, _radius_change_speed: f64, _rules: &Rules, col:&mut(&mut bool, &mut Vec3)) {
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
                *col.0 = true;
                *col.1 = *col.1 * 0.2 + _b.velocity3() * 0.8;
            }
        }
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


    fn simpleRobotBallColideStep(_robot: &Entity3, _ball: &Entity3, _radius_change_speed: f64,_rules: &Rules) -> Vec3 {
        let delta_position = _ball.position3() - _robot.position3();
        let distance = delta_position.len();
        let penetration = _robot.radius() + _ball.radius() - distance;
        let a_pos = _robot.position3();
        let b_pos = _ball.position3();
        let a_vel = _robot.velocity3();
        let b_vel = _ball.velocity3();
        if penetration > 0.0 {
            let k_a = (1.0 / _robot.mass()) / ((1.0 / _ball.mass()) + (1.0  / _ball.mass()));
            let k_b = (1.0 / _ball.mass()) / ((1.0 / _robot.mass()) + (1.0  / _ball.mass()));
            let normal = delta_position.normalize();
            let delta_vel = (_ball.velocity3() - _robot.velocity3()).inner_product(&normal) - _radius_change_speed;
            //+ _b.radius_change_speed() - _a.radius_change_speed();
            if delta_vel < 0.0 {
                let impulse = normal * (1.0 + (_rules.MIN_HIT_E + _rules.MAX_HIT_E)/2.0) * delta_vel;
                let finalVel = b_vel - (impulse * k_b);
                finalVel

            } else {
                b_vel
            }
        } else {
                b_vel
        }
    }

    fn bestPlaceOnBallForKick(finalVel : Vec3 ,_robot: &Entity3, _ball: &Entity3, _radius_change_speed: f64,_rules: &Rules) -> Vec3 {
            let _result = Vec3::new(5000.0,5000.0,5000.0);
            let _finalVelNormm = finalVel.normalize();
            let mut _x = 0.0;
            let mut _y = 0.0;
            let mut _z = 0.0;
            let mut _theta = 0.0;
            let mut _phi = 0.0;
            let mut _impulseVec = Vec3::new(0.0,0.0,0.0);
            let _radius = _ball.radius() + _robot.radius();
            for i in (0..360).step_by(5) {
                for j in (0..360).step_by(5) {
                    _theta = (i as f64) * RAD2DEG;
                    _phi = (j as f64) * RAD2DEG;
                    _x = _radius * (_phi.sin())*(_theta.cos());
                    _y = _radius * (_phi.cos())*(_theta.sin());
                    _z = _radius * (_phi.cos());


                }
            }
            return _result;
    }

    fn collide_with_arena(_e: &mut Entity3, _radius_change_speed: f64, _rules: &Rules) -> Vec3 {
        let (distance, normal) = DAN::dan_to_arena(&_e.position3(), _rules);
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
        let mut vel = _e.velocity3();
        _e.set_velocity(&Self::clamp(&vel, _rules.MAX_ENTITY_SPEED));
        vel = _e.velocity3();
        _e.set_position(&(pos + (vel * delta_time)));
        let h = _e.height3();
        _e.set_height(h - (_rules.GRAVITY * delta_time * delta_time / 2.0));
        _e.set_velocity_h(vel.h - (_rules.GRAVITY * delta_time));
    }

    fn update(_me : &mut Robot, _ball: &mut Ball, _action: &Action, _rules: &Rules, delta_time: f64, col:&mut(&mut bool, &mut Vec3)) {
            if _me.touch {
                let mut target_vel = Self::clamp(&_action.target_vel(), _rules.ROBOT_MAX_GROUND_SPEED);
                target_vel -= _me.touch_normal() * _me.touch_normal().inner_product(&target_vel);
                let target_vel_change = target_vel - _me.velocity3();
                if target_vel_change.len() > 0.0 {
                    let acc = _rules.ROBOT_ACCELERATION * _me.touch_normal_y.unwrap().max(0.0);
                    let robot_vel = _me.velocity3();
                    _me.set_velocity(&(robot_vel + Self::clamp(&(target_vel_change.normalize() * acc * delta_time), target_vel_change.len())));
                }
                // TODO : USE NITRO
            }
            Self::move_e(_me, delta_time, _rules);
            _me.radius = _rules.ROBOT_MIN_RADIUS + (_rules.ROBOT_MAX_RADIUS - _rules.ROBOT_MIN_RADIUS) * _action.jump_speed / _rules.ROBOT_MAX_JUMP_SPEED;

            Self::move_e(_ball, delta_time, _rules);
            Self::collide_entities_col(_me, _ball, _action.jump_speed, _rules, col);
            let collision_normal = Self::collide_with_arena(_me, _action.jump_speed, _rules);
            if ! collision_normal.is_valid() {
                _me.touch = false;
            } else {
                _me.touch = true;
                _me.set_touch_normal(&collision_normal);
            }
            Self::collide_with_arena(_ball, 0.0, _rules);
    }

    fn tick(_me : &mut Robot, _ball: &mut Ball, _action: &Action, _rules: &Rules) -> (bool, Vec3){
        let delta_time = 1.0 / _rules.TICKS_PER_SECOND as f64;
        let mut c = 0;
        let mut v = Vec3::default();
        for _ in 0 .. 99 {
            let mut collide = false;
            let mut col_vel = Vec3::default();
            Self::update(_me, _ball, _action, _rules, delta_time / 100.0 as f64, &mut (&mut collide, &mut col_vel));
            if collide {
                println!("COL: {:?}", _action);
                c += 1;
                v += col_vel;
            }
        }
        (c > 20, v * (1.0 / f64::from(c)))
    }

    fn update_ball(_ball: &mut Ball, _rules: &Rules, delta_time: f64) {
            Self::move_e(_ball, delta_time, _rules);
            Self::collide_with_arena(_ball, 0.0, _rules);
    }

    fn tick_ball(_ball: &mut Ball, _rules: &Rules, _time: f64) {
        let delta_time = _time / _rules.TICKS_PER_SECOND as f64;
        for _ in 0 .. 100 - 1 {
            Self::update_ball(_ball, _rules, delta_time / 100 as f64);
        }
    }

    fn update_game(_id : i32, _game : &mut Game, _rules: &Rules, delta_time: f64) {

        Self::move_e(&mut _game.ball, delta_time, _rules);
        Self::collide_with_arena(&mut _game.ball, 0.0, _rules);

        for _me in &mut _game.robots {
            if _me.id == _id || _me.touch {
                continue;
            }
            if _me.touch {
                let mut target_vel = Self::clamp(&_me.velocity3(), // <- this should be action.target_vel()
                 _rules.ROBOT_MAX_GROUND_SPEED);
                target_vel -= _me.touch_normal() * _me.touch_normal().inner_product(&target_vel);
                let target_vel_change = target_vel - _me.velocity3();
                if target_vel_change.len() > 0.0 {
                    let acc = _rules.ROBOT_ACCELERATION * _me.touch_normal_y.unwrap().max(0.0);
                    let robot_vel = _me.velocity3();
                    _me.set_velocity(&(robot_vel + Self::clamp(&(target_vel_change.normalize() * acc * delta_time), target_vel_change.len())));
                }
                // TODO : USE NITRO
            }

            Self::move_e(_me, delta_time, _rules);
            _me.radius = _rules.ROBOT_MIN_RADIUS + (_rules.ROBOT_MAX_RADIUS - _rules.ROBOT_MIN_RADIUS) * 0.0 // <- This should be action.jump_speed
             / _rules.ROBOT_MAX_JUMP_SPEED;
            Self::collide_entities(_me, &mut _game.ball, 0.0, _rules);
            let collision_normal = Self::collide_with_arena(_me, 0.0, _rules);
            if ! collision_normal.is_valid() {
                _me.touch = false;
            } else {
                _me.touch = true;
                _me.set_touch_normal(&collision_normal);
            }
        }

        // for i in 0 .. _game.robots.len() {
        //     for j in 0 .. _game.robots.len() {
        //         let mut jump_speed = 0.0;
        //         if _game.robots[i].touch && !_game.robots[j].touch {
        //             jump_speed = _rules.ROBOT_MAX_JUMP_SPEED;
        //         } else if !_game.robots[i].touch && _game.robots[j].touch {
        //             jump_speed = -_rules.ROBOT_MAX_JUMP_SPEED;
        //
        //         }
        //         let mut a = _game.robots[j].clone();
        //         Self::collide_entities(&mut _game.robots[i], &mut a, jump_speed, _rules);
        //     }
        // }



    }

    fn tick_game(_id: i32, _game: &mut Game, _rules: &Rules) {
        let delta_time = 1.0 / _rules.TICKS_PER_SECOND as f64;
        for _ in 0 .. 50 {
            Self::update_game(_id, _game, _rules, delta_time / 50 as f64);
        }
    }

    fn get_ball_path(_me: i32, _game: &Game, _rules : &Rules) -> [Ball;BALL_PREDICTION_TICKS] {
        let mut res = [Ball::default(); BALL_PREDICTION_TICKS];
        let mut game = &mut _game.clone();
        for i in res.iter_mut() {
            Self::tick_game(_me, &mut game, _rules);
            *i = game.ball;
        }
        res
    }
}
