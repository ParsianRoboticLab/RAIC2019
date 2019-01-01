#[derive(Debug)]
enum GameMode {
	NONE,
	DEF,
	NORMAL,
	OFF,
}

#[derive(Debug)]
enum Role {
	NONE = 0,
	GK = 1,
	DEF = 2,
	SUP = 3,
    OFF = 4,
}

struct Coach {
    current_state: GameMode,
    last_state: GameMode,
	last_roles: Role,
}

impl Coach {
    fn default() -> Self {
        Self {current_state:GameMode::NONE, last_state:GameMode::NONE, last_roles:Role::NONE}
    }

    fn choose_mode(&mut self, _game: &Game, _rules: &Rules) {
        self.current_state = GameMode::NORMAL;
        self.last_state = GameMode::NORMAL;
    }

    fn find_role(&mut self, me: &Robot, _game: &Game, _rules: &Rules) -> Role {

		let our_goal = Vec2::new(0.0, -_rules.arena.depth/2.0);
        for _robot in &_game.robots {
            if _robot.is_teammate && _robot.id != me.id {
				let robot = _robot as &Entity;
				let ball = Vec2::new(_game.ball.x, _game.ball.z);
				let mut add = 0.0;
				match self.last_roles {
					Role::GK => add += 20.0,
					_ => add += 0.0,
				}

				let f_m  = 3.0 * (me as &Entity).position().dist(ball - Vec2::new(0.0, -4.0)) - (me as &Entity).position().dist(our_goal);
				let f_r  = 3.0 * robot.position().dist(ball - Vec2::new(0.0, -4.0)) - robot.position().dist(our_goal);
                if f_r > f_m - add{
					self.last_roles = Role::OFF;
                    return Role::OFF
                } else {
					self.last_roles = Role::GK;
                    return Role::GK
                }
            }
        }
        Role::NONE
    }
}
