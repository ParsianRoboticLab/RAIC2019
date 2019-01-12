impl Default for Arena {
    fn default() -> Self {
        Self {
            width: 0.,
            height: 0.,
            depth: 0.,
            bottom_radius: 0.,
            top_radius: 0.,
            corner_radius: 0.,
            goal_top_radius: 0.,
            goal_width: 0.,
            goal_height: 0.,
            goal_depth: 0.,
            goal_side_radius: 0.,
        }
    }
}

impl Default for Rules {
    fn default() -> Self {
        Self {
            max_tick_count: 0,
            arena: Arena{..Default::default()},
            team_size: 0,
            seed: 0,
            ROBOT_MIN_RADIUS: 0.,
            ROBOT_MAX_RADIUS: 0.,
            ROBOT_MAX_JUMP_SPEED: 0.,
            ROBOT_ACCELERATION: 0.,
            ROBOT_NITRO_ACCELERATION: 0.,
            ROBOT_MAX_GROUND_SPEED: 0.,
            ROBOT_ARENA_E: 0.,
            ROBOT_RADIUS: 0.,
            ROBOT_MASS: 0.,
            TICKS_PER_SECOND: 0,
            MICROTICKS_PER_TICK: 0,
            RESET_TICKS: 0,
            BALL_ARENA_E: 0.,
            BALL_RADIUS: 0.,
            BALL_MASS: 0.,
            MIN_HIT_E: 0.,
            MAX_HIT_E: 0.,
            MAX_ENTITY_SPEED: 0.,
            MAX_NITRO_AMOUNT: 0.,
            START_NITRO_AMOUNT: 0.,
            NITRO_POINT_VELOCITY_CHANGE: 0.,
            NITRO_PACK_X: 0.,
            NITRO_PACK_Y: 0.,
            NITRO_PACK_Z: 0.,
            NITRO_PACK_RADIUS: 0.,
            NITRO_PACK_AMOUNT: 0.,
            NITRO_PACK_RESPAWN_TICKS: 0,
            GRAVITY: 0.,
        }
    }
}


impl Default for Robot {
    fn default() -> Self {
        Self {
           id: 0,
           player_id: 0,
           is_teammate: false,
           x: 0.0,
           y: 0.0,
           z: 0.0,
           velocity_x: 0.0,
           velocity_y: 0.0,
           velocity_z: 0.0,
           radius: 0.0,
           nitro_amount: 0.0,
           touch: false,
           touch_normal_x: Some(0.0),
           touch_normal_y: Some(0.0),
           touch_normal_z: Some(0.0),
        }
    }
}

impl Default for Ball {
    fn default() -> Self {
    Self{
        x: 0.0,
        y: 0.0,
        z: 0.0,
        velocity_x: 0.0,
        velocity_y: 0.0,
        velocity_z: 0.0,
        radius: 0.0,
    }
    }
}

impl Default for Game {
    fn default() -> Self {
        Self{
            current_tick: 0,
            players: vec![],
            robots: Vec::new(),
            nitro_packs: Vec::new(),
            ball: Ball{..Default::default()}
        }
    }
}

trait VEC {
    fn target_vel(&self) -> Vec3;
}

impl VEC for Action {
    fn target_vel(&self) -> Vec3 {
        Vec3{
            x: self.target_velocity_x,
            y: self.target_velocity_z,
            h: self.target_velocity_y,
        }
    }
}

impl Copy for Ball {

}
