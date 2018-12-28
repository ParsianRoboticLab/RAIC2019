use crate::model::*;
use crate::strategy::Strategy;



include!("pid.rs");
include!("vec2.rs");
include!("def.rs");
include!("entity.rs");
include!("coach.rs");

pub struct MyStrategy{
    coach : Coach,
    me : Robot,
    rules: Rules,
    game: Game,
    action: Action,
    posPID : PID
}

impl Default for MyStrategy {
    fn default() -> Self {
        Self {
            coach: Coach::default(),
            me: Robot{},
            rules: Rules{..Default::default()},
            game: Game{..Default::default()},
            action: Action{..Default::default()},
            posPID: PID::new(15.0,0.0,0.0),
        }
    }
}

impl Strategy for MyStrategy {
    fn act(&mut self, me: &Robot, _rules: &Rules, _game: &Game, _action: &mut Action) {
    	// Choose Main Strategy (Coach) 1. DEF, 2. NORMAL, 3. OFF
        self.me = me.clone();
        self.rules = _rules.clone();
        self.game = _game.clone();

        self.coach.choose_mode(_game, _rules);
    	// Choose My Role 1. GK, 2. DEF, 3. OFF 4. SUP
        let my_role = self.coach.find_role(me, _game, _rules);
    	// Execute My Role
        let oppGoal = Vec2::new(0.0, self.rules.arena.depth/2.0);
        match my_role {
            Role::NONE =>  println!("No Role is Selected"),
            Role::GK   =>  self.gk(),
            Role::DEF  =>  println!("The color is Red!"),
            Role::SUP  =>  println!("The color is Red!"),
            Role::OFF  =>  self.pm(&oppGoal),
        }
    	// Fill Action (Control)
        *_action = self.action;
    	// println!("Role: {:#?}", my_role);
    	// println!("Action: {:#?}", _action);

    }

}

impl MyStrategy {
    fn gk(&mut self) {
        let mut x = self.game.ball.position().x;
        if x < -self.rules.arena.goal_width/2.0 {
            x = -self.rules.arena.goal_width/2.0;
        } else if x > self.rules.arena.goal_width/2.0 {
            x = self.rules.arena.goal_width/2.0;
        }
        let v = Vec2::new(x, -self.rules.arena.depth/2.0);
        self.gtp(&v);
    }

    fn pm(&mut self, target: &Vec2) {
        let ballpos = self.game.ball.position();
        let robotpos = self.me.position() + self.me.velocity();
        let norm = (ballpos - *target).normalize();
        let behind = ballpos + (ballpos - *target).normalize()*3.0;
        let goal = ballpos - (ballpos - *target).normalize()*3.0;
        let avoid = ballpos + Vec2::new(4.0, 0.0);
        /*if robotpos.dist(*target) < ballpos.dist(*target) - 5.0{
            self.gtp(&avoid);
        }
        else */if robotpos.dist(behind) > 10.0 {
            self.gtp(&behind);
            println!("SALAM");
        } else {
            println!("BYE");
            self.gtp(&goal);
        }

    }

    fn gtp(&mut self, target: &Vec2) {
        let dist = self.me.position().dist(*target);
        let diff = *target - self.me.position();
        let angle = (diff.y).atan2(diff.x);
        let a  = self.posPID.run(dist);
        println!("{}**{}**{}", dist, angle, a);
        self.set_robot_vel(angle, a);

    }

    fn set_robot_vel(&mut self, angle : f64, vel: f64) {
        self.action = Action {
            target_velocity_x: vel*angle.cos(),
            target_velocity_y: 0.0,
            target_velocity_z: vel*angle.sin(),
            jump_speed: 0.0,
            use_nitro: false,
        }
    }
}
