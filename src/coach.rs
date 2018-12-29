#[derive(Debug)]
enum GameMode {
	NONE,
	DEF,
	NORMAL,
	OFF,
}

#[derive(Debug)]
enum Role {
	NONE,
	GK,
	DEF,
	SUP,
    OFF,
}

struct Coach {
    current_state: GameMode,
    last_state: GameMode,
}

impl Coach {
    fn default() -> Self {
        Self {current_state:GameMode::NONE, last_state:GameMode::NONE}
    }

    fn choose_mode(&mut self, _game: &Game, _rules: &Rules) {
        self.current_state = GameMode::NORMAL;
        self.last_state = GameMode::NORMAL;
    }

    fn find_role(&mut self, me: &Robot, _game: &Game, _rules: &Rules) -> Role {
		let our_goal = Vec2::new(0.0, -_rules.arena.depth/2.0);
        for _robot in &_game.robots {
			let robot = _robot as &Entity;
			let ball = Vec2::new(_game.ball.x, _game.ball.z);
            if _robot.is_teammate && _robot.id != me.id {
                if robot.position().dist(ball - Vec2::new(0.0, -4.0)) > (me as &Entity).position().dist(ball - Vec2::new(0.0, -4.0)) &&
				 robot.position().dist(our_goal) < (me as &Entity).position().dist(our_goal) + 10.0 {
                     return Role::OFF
                } else {
                    return Role::GK
                }
            }
        }
        Role::NONE
    }
}
