struct DAN {

}

impl DAN {
    fn dan_to_plane(point: &Vec3, point_on_plane: &Vec3, plane_normal: &Vec3) -> (f64,Vec3) {
        ((*point - *point_on_plane).inner_product(&plane_normal) , *plane_normal)
    }

    fn dan_to_sphere_inner(point: &Vec3, sphere_center: &Vec3, sphere_radius: &f64) -> (f64,Vec3) {
        (sphere_radius - (*point - *sphere_center).len(),(*sphere_center - *point).normalize())
    }

    fn dan_to_sphere_outer(point: &Vec3, sphere_center: &Vec3, sphere_radius: &f64) -> (f64,Vec3) {
        ((*point - *sphere_center).len() - sphere_radius,(*point - *sphere_center).normalize())

    }

    fn min_dan(a: (f64,Vec3), b :(f64,Vec3)) -> (f64,Vec3) {
        if a.0 < b.0 {
            a
        } else {
            b
        }
    }

    fn clamp(_input : f64 , _min : f64 , _max : f64) -> f64{
        if _input > _max {
            _max
        }
        else if _input < _min {
            _min
        } else {
            _input
        }

    }

    fn dan_to_arena_quarter(point: &Vec3, _rules: &Rules) -> (f64,Vec3) {
        // Ground
        let mut dan = Self::dan_to_plane(point, &Vec3::new(0.0, 0.0, 0.0), &Vec3::new(0.0, 1.0, 0.0));
        // Ceiling
        dan = Self::min_dan(dan,
            Self::dan_to_plane(point, &Vec3::new(0.0, _rules.arena.height, 0.0),
             &Vec3::new(0.0, -1.0, 0.0)));

        // Side x
        dan = Self::min_dan(dan,
            Self::dan_to_plane(point, &Vec3::new(_rules.arena.width / 2.0, 0.0, 0.0),
            &Vec3::new(-1.0, 0.0, 0.0)));

        // Side z (goal)
        dan = Self::min_dan(dan,
            Self::dan_to_plane(point, &Vec3::new(0.0, 0.0, (_rules.arena.depth / 2.0)
            + _rules.arena.goal_depth),
            &Vec3::new(0.0, 0.0, -1.0)));

        // Side z
        let v = Vec2::new(point.x, point.h) - Vec2::new((_rules.arena.goal_width / 2.0)
        - _rules.arena.goal_top_radius,
         _rules.arena.goal_height - _rules.arena.goal_top_radius);
        if point.x >= (_rules.arena.goal_width / 2.0) + _rules.arena.goal_side_radius
        || point.y >= _rules.arena.goal_height + _rules.arena.goal_side_radius
        || (v.x > 0.0 && v.y > 0.0 && v.len() >= _rules.arena.goal_top_radius
                                                + _rules.arena.goal_side_radius) {
            dan = Self::min_dan(dan , Self::dan_to_plane(point, &Vec3::new(0.0, 0.0, _rules.arena.depth / 2.0), &Vec3::new(0.0, 0.0, -1.0)));
        }

        //Side x & ceiling (goal)
        if point.y >= (_rules.arena.depth / 2.0) + _rules.arena.goal_side_radius {
            // x
            dan = Self::min_dan(dan, Self::dan_to_plane(point,
                 &Vec3::new(_rules.arena.goal_width / 2.0, 0.0, 0.0),
                 &Vec3::new(-1.0, 0.0, 0.0)));
            // h
            dan = Self::min_dan(dan, Self::dan_to_plane(point, &Vec3::new(0.0,
                 _rules.arena.goal_height, 0.0),
                 &Vec3::new(0.0, -1.0, 0.0)));
        }

        // Goal back corners
        if point.y > (_rules.arena.depth / 2.0)
        + _rules.arena.goal_depth
        - _rules.arena.bottom_radius {
            dan = Self::min_dan(dan, Self::dan_to_sphere_inner(point,
                 &Vec3::new(Self::clamp(point.x, _rules.arena.bottom_radius - (_rules.arena.goal_width / 2.0),
                 (_rules.arena.goal_width / 2.0) - _rules.arena.bottom_radius),
                 Self::clamp(point.h,_rules.arena.bottom_radius,_rules.arena.goal_height - _rules.arena.goal_top_radius),
                 (_rules.arena.depth / 2.0) + _rules.arena.goal_depth - _rules.arena.bottom_radius),
                 &_rules.arena.bottom_radius));
        }
        // Corner
        if point.x > (_rules.arena.width / 2.0) - _rules.arena.corner_radius && point.y > (_rules.arena.depth / 2.0) - _rules.arena.corner_radius {
            dan = Self::min_dan(dan, Self::dan_to_sphere_inner(point, &Vec3::new((_rules.arena.width / 2.0) - _rules.arena.corner_radius, point.h, (_rules.arena.depth / 2.0) - _rules.arena.corner_radius), &_rules.arena.corner_radius));
        }
        // Goal outer corner
        if point.y < (_rules.arena.depth / 2.0) + _rules.arena.goal_side_radius {
            // Side x
            if point.x < (_rules.arena.goal_width / 2.0) + _rules.arena.goal_side_radius {
                dan = Self::min_dan(dan, Self::dan_to_sphere_outer(point,
                    &Vec3::new((_rules.arena.goal_width / 2.0) + _rules.arena.goal_side_radius,
                    point.h,
                    (_rules.arena.depth / 2.0) + _rules.arena.goal_side_radius),
                    &_rules.arena.goal_side_radius));
            }
            // Ceiling
            if point.h < _rules.arena.goal_height + _rules.arena.goal_side_radius {
                dan = Self::min_dan(dan,
                    Self::dan_to_sphere_outer(point, &Vec3::new(point.x,
                        _rules.arena.goal_height + _rules.arena.goal_side_radius,
                        (_rules.arena.depth / 2.0) + _rules.arena.goal_side_radius
                    ),
                    &_rules.arena.goal_side_radius));
            }
            // Top corner
            let o = Vec2::new((_rules.arena.goal_width / 2.0) - _rules.arena.goal_top_radius,
                            _rules.arena.goal_height - _rules.arena.goal_top_radius);
            let v = Vec2::new(point.x, point.h) - o;
            if v.x > 0.0 && v.y > 0.0 {
                let o = o + v.normalize() * (_rules.arena.goal_top_radius + _rules.arena.goal_side_radius);
                dan = Self::min_dan(dan, Self::dan_to_sphere_outer(
                    point,
                    &Vec3::new(o.x, o.y, (_rules.arena.depth / 2.0) + _rules.arena.goal_side_radius),
                    &_rules.arena.goal_side_radius));
            }
        }
        // Goal inside top corners
        if point.y > (_rules.arena.depth / 2.0) + _rules.arena.goal_side_radius
        && point.h > _rules.arena.goal_height - _rules.arena.goal_top_radius {
            // Side x
            if point.x > (_rules.arena.goal_width / 2.0) - _rules.arena.goal_top_radius {
                dan = Self::min_dan(dan, Self::dan_to_sphere_inner(
                    point,
                    &Vec3::new(
                        (_rules.arena.goal_width / 2.0) - _rules.arena.goal_top_radius,
                        _rules.arena.goal_height - _rules.arena.goal_top_radius,
                        point.y
                    ),
                    &_rules.arena.goal_top_radius));
                }
            // Side z
            if point.y > (_rules.arena.depth / 2.0) + _rules.arena.goal_depth - _rules.arena.goal_top_radius {
                dan = Self::min_dan(dan, Self::dan_to_sphere_inner(point,&Vec3::new(point.x,
                    _rules.arena.goal_height - _rules.arena.goal_top_radius,
                    (_rules.arena.depth / 2.0) + _rules.arena.goal_depth - _rules.arena.goal_top_radius
                ),
                &_rules.arena.goal_top_radius));
            }
        }
        if point.h < _rules.arena.bottom_radius {
            // Side x
            if point.x > (_rules.arena.width / 2.0) - _rules.arena.bottom_radius {
                dan = Self::min_dan(dan, Self::dan_to_sphere_inner(
                    point,
                    &Vec3::new(
                        (_rules.arena.width / 2.0) - _rules.arena.bottom_radius,
                        _rules.arena.bottom_radius,
                        point.y
                    ),
                    &_rules.arena.bottom_radius));
                }
            // Side z
            if point.y > (_rules.arena.depth / 2.0) - _rules.arena.bottom_radius
            && point.x >= (_rules.arena.goal_width / 2.0) + _rules.arena.goal_side_radius {
                dan = Self::min_dan(dan, Self::dan_to_sphere_inner(
                    point,
                    &Vec3::new(
                        point.x,
                        _rules.arena.bottom_radius,
                        (_rules.arena.depth / 2.0) - _rules.arena.bottom_radius
                    ),
                    &_rules.arena.bottom_radius));
                }
            // Side z (goal)
            if point.y > (_rules.arena.depth / 2.0) + _rules.arena.goal_depth - _rules.arena.bottom_radius {
                dan = Self::min_dan(dan, Self::dan_to_sphere_inner(
                    point,
                    &Vec3::new(
                        point.x,
                        _rules.arena.bottom_radius,
                        (_rules.arena.depth / 2.0) + _rules.arena.goal_depth - _rules.arena.bottom_radius),
                        &_rules.arena.bottom_radius));
            }
            // Goal outer corner
            let o = Vec2::new(
                (_rules.arena.goal_width / 2.0) + _rules.arena.goal_side_radius,
                (_rules.arena.depth / 2.0) + _rules.arena.goal_side_radius
            );
            let v = Vec2::new(point.x, point.y) - o;
            if v.x < 0.0 && v.y < 0.0
            && v.len() < _rules.arena.goal_side_radius + _rules.arena.bottom_radius{
                let o = o + v.normalize() * (_rules.arena.goal_side_radius + _rules.arena.bottom_radius);
                dan = Self::min_dan(dan, Self::dan_to_sphere_inner(
                    point,
                    &Vec3::new(o.x, _rules.arena.bottom_radius, o.y),
                    &_rules.arena.bottom_radius));
                }
            // Side x (goal)
            if point.y >= (_rules.arena.depth / 2.0) + _rules.arena.goal_side_radius
            && point.x > (_rules.arena.goal_width / 2.0) - _rules.arena.bottom_radius {
                dan = Self::min_dan(dan, Self::dan_to_sphere_inner(
                    point,
                    &Vec3::new(
                        (_rules.arena.goal_width / 2.0) - _rules.arena.bottom_radius,
                        _rules.arena.bottom_radius,
                        point.y
                    ),
                    &_rules.arena.bottom_radius));
                }
            // Corner
            if point.x > (_rules.arena.width / 2.0) - _rules.arena.corner_radius
            && point.y > (_rules.arena.depth / 2.0) - _rules.arena.corner_radius {
                let corner_o = Vec2::new(
                    (_rules.arena.width / 2.0) - _rules.arena.corner_radius,
                    (_rules.arena.depth / 2.0) - _rules.arena.corner_radius
                );
                let n = Vec2::new(point.x, point.y) - corner_o;
                let dist = n.len();
                if dist > _rules.arena.corner_radius - _rules.arena.bottom_radius {
                    let n = n * (1.0/dist);
                    let o2 = corner_o + n * (_rules.arena.corner_radius - _rules.arena.bottom_radius);
                    dan = Self::min_dan(dan, Self::dan_to_sphere_inner(
                        point,
                        &Vec3::new(o2.x, _rules.arena.bottom_radius, o2.y),
                        &_rules.arena.bottom_radius));
                    }
                }
            }
            // Ceiling corners
            if point.h > _rules.arena.height - _rules.arena.top_radius {
                // Side x
                if point.x > (_rules.arena.width / 2.0) - _rules.arena.top_radius{
                    dan = Self::min_dan(dan, Self::dan_to_sphere_inner(
                        point,
                        &Vec3::new(
                            (_rules.arena.width / 2.0) - _rules.arena.top_radius,
                            _rules.arena.height - _rules.arena.top_radius,
                            point.y,
                        ),
                        &_rules.arena.top_radius));
                    }
            // Side z
            if point.y > (_rules.arena.depth / 2.0) - _rules.arena.top_radius{
                dan = Self::min_dan(dan, Self::dan_to_sphere_inner(
                    point,
                    &Vec3::new(
                        point.x,
                        _rules.arena.height - _rules.arena.top_radius,
                        (_rules.arena.depth / 2.0) - _rules.arena.top_radius,
                    ),
                    &_rules.arena.top_radius));
                }

                // Corner
                if point.x > (_rules.arena.width / 2.0) - _rules.arena.corner_radius && point.y > (_rules.arena.depth / 2.0) - _rules.arena.corner_radius {
                    let corner_o = Vec2::new(
                        (_rules.arena.width / 2.0) - _rules.arena.corner_radius,
                        (_rules.arena.depth / 2.0) - _rules.arena.corner_radius
                    );
                    let dv = Vec2::new(point.x, point.y) - corner_o;
                    if dv.len() > _rules.arena.corner_radius - _rules.arena.top_radius {
                        let n = dv.normalize();
                        let o2 = corner_o + n * (_rules.arena.corner_radius - _rules.arena.top_radius);
                        dan = Self::min_dan(dan, Self::dan_to_sphere_inner(
                            point,
                            &Vec3::new(o2.x, _rules.arena.height - _rules.arena.top_radius, o2.y),
                            &_rules.arena.top_radius));
                }
            }
        }
        dan
    }

    fn dan_to_arena(point: &Vec3, _rules: &Rules) -> (f64,Vec3){
        let negate_x = point.x < 0.0;
        let negate_z = point.y < 0.0;
        let mut v = *point;
        if negate_x {
            v.x *= -1.0;
        }
        if negate_z {
            v.y *= -1.0;
        }

        let mut result = Self::dan_to_arena_quarter(&v, _rules);
        if negate_x {
            (result.1).x = -(result.1).x;
        }
        if negate_z {
            (result.1).y = -(result.1).y;
        }
        result
    }
}
