use crate::model::*;
use crate::strategy::Strategy;

const TWO_PI : f64 = 2.0 * std::f64::consts::PI;
const EPSILON : f64 = 1.0e-6;
include!("pid.rs");
include!("vec2.rs");
include!("def.rs");
include!("entity.rs");
include!("coach.rs");
include!("angdeg.rs");
include!("seg2.rs");
include!("line2.rs");


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
            me: Robot{..Default::default()},
            rules: Rules{..Default::default()},
            game: Game{..Default::default()},
            action: Action{..Default::default()},
            posPID: PID::new(10.0,0.0,0.0),
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
        let ball_pos = self.game.ball.position();
        let goal_line = Seg2{
            origin:   Vec2{x:-self.rules.arena.goal_width/2.0, y:0.0},
            terminal: Vec2{x: self.rules.arena.goal_width/2.0, y:0.0}
        };
        let ball_seg = Seg2::new3(self.game.ball.position(), 100.0, self.game.ball.velocity().th());
        let mut target = Vec2{x:0.0, y:-self.rules.arena.depth/2.0};
        if self.game.ball.velocity().y < -1.0 { // KICK
            target = goal_line.intersection(ball_seg);
        } else if self.game.ball.position().y  < 1.0 {
            let mut x = self.game.ball.position().x;
            if x < -self.rules.arena.goal_width/2.0 {
                x = -self.rules.arena.goal_width/2.0;
            } else if x > self.rules.arena.goal_width/2.0 {
                x = self.rules.arena.goal_width/2.0;
            }
            target = Vec2::new(x, -self.rules.arena.depth/2.0);

        }
        self.gtp(&target);
    }

    fn kick(&mut self, target: &Vec2)  {
        let ballpos = self.game.ball.position();
        let robotpos = self.me.position();
        let finalDir = (*target - ballpos).th().deg();
        let mut idealPath = (ballpos - robotpos).th().deg();
        let mut movementDir = ((ballpos - robotpos).th().deg() - finalDir)*180.0/3.1415;
        let ballVel = self.game.ball.velocity();
        if movementDir >= 180.0 {
            movementDir -= 360.0;
        }
        if movementDir < -180.0 {
            movementDir += 360.0;
        }
        println!("movementDir {}", movementDir );

        let mut shift = 0.0;

        if movementDir.abs() < 5.0 {
            shift = 0.0;
        } else if movementDir > 0.0 {
            shift = 30.0;
        } else {
            shift = -30.0;
        }
        let mut jump = 0.0;
        let robotCurrentPath = self.me.velocity().th().deg();
        if (robotpos.dist(ballpos) < (self.me.radius + self.game.ball.radius + 1.5)) && (movementDir.abs() < 15.0) && (self.game.ball.height() < 3.0)  {
            idealPath = (*target - robotpos).th().deg();
            if ((robotCurrentPath - idealPath).abs() * RAD2DEG) < 20.0 {
                jump = 1000.0;
            } else {
                jump = 0.0;
            }
        } else {
            jump = 0.0;
            idealPath = idealPath + shift*3.1415/180.0 ;
        }
        println!("ball height {}", self.game.ball.height());

        self.set_robot_vel(idealPath ,1000.0,jump);


    }
    fn pm(&mut self, target: &Vec2) {
        self.kick(target);

    }

    fn gtp(&mut self, target: &Vec2) {
        let dist = self.me.position().dist(*target);
        let diff = *target - self.me.position();
        let angle = (diff.y).atan2(diff.x);
        let a  = self.posPID.run(dist);
        println!("{}**{}**{}", dist, angle, a);
        self.set_robot_vel(angle, a , 0.0);

    }

    fn set_robot_vel(&mut self, angle : f64, vel: f64, jump : f64) {
        self.action = Action {
            target_velocity_x: vel*angle.cos(),
            target_velocity_y: 0.0,
            target_velocity_z: vel*angle.sin(),
            jump_speed: self.action.jump_speed,
            use_nitro: self.action.use_nitro,
        }
    }
}
