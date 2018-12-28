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
            posPID: PID::new(5.0,0.0,0.0),
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

fn get_bisect(seg: &Seg2, vec: &Vec2) -> Seg2{
    let ang = angle_of(&seg.origin(), vec, &seg.terminal())/AngDeg::new(2.0);
    let v2 = seg.origin().rotateVector(-ang.deg());
    let s = Seg2::new(*vec, v2);
    let v3 = s.intersection(*seg);
    println!("AAA {:?} {:?}",ang, v3);
    Seg2::new(*vec, v3)
}


impl MyStrategy {


    fn gk(&mut self) {
        let ball_pos = self.game.ball.position();
        let goal_line = Seg2{
            origin:   Vec2{x: self.rules.arena.goal_width/2.0, y:-self.rules.arena.depth/2.0},
            terminal: Vec2{x:-self.rules.arena.goal_width/2.0, y:-self.rules.arena.depth/2.0}
        };
        let ball_seg = Seg2::new(self.game.ball.position(), self.game.ball.velocity()*100.0);
        let biset = get_bisect(&goal_line, &ball_pos);
        let mut target = Vec2{x:0.0, y:-self.rules.arena.depth/2.0};
        println!("DFAS");
        if self.game.ball.velocity().y < -1.0 { // KICK
            target = goal_line.intersection(ball_seg);
            if target.is_valid() {
                target = Vec2::new(ball_pos.x, -self.rules.arena.depth/2.0);
            }
        } else if self.game.ball.position().y  < 1.0 {
            target = biset.terminal();
        }
        if target.x < -self.rules.arena.goal_width/2.0 + 1.5 {
            target.x = -self.rules.arena.goal_width/2.0 + 1.5;
        } else if target.x > self.rules.arena.goal_width/2.0 - 1.5{
            target.x = self.rules.arena.goal_width/2.0 - 1.5;
        }
        self.gtp(&target);

        if (ball_pos.dist(self.me.position()) < 4.0 && self.game.ball.height() > 3.0) {
            self.action.jump_speed = self.rules.ROBOT_MAX_JUMP_SPEED;
        } else {
            self.action.jump_speed = 0.0;
        }

    }

    fn ballTouchPrediction(&mut self) -> Vec2 {
        let gravity = self.rules.GRAVITY;
        let ballpos = self.game.ball.position();
        let ballHeight = self.game.ball.height();
        let ballVel = self.game.ball.velocity();
        let ballhVel = self.game.ball.hVel();
        let timeToTouchTheField = (ballhVel + (ballhVel*ballhVel + 2.0*gravity*(ballHeight - self.game.ball.radius)).sqrt()) / gravity;

        println!("ballVelx: {}, ballVelY : {} , ballVelLen : {} , time : {}",ballVel.x,ballVel.y,ballVel.len(),timeToTouchTheField );
        let lenTravel = timeToTouchTheField * ballVel.len();
        if ballVel.len() == 0.0 {
            ballpos
        }
        else {
            ballpos + ballVel.normalize() * lenTravel
        }
    }
    fn kick(&mut self, target: &Vec2)  {
          let ballpos = self.game.ball.position();
        let robotpos = self.me.position();
        let finalDir = (*target - ballpos).th();
        let mut idealPath = (ballpos - robotpos).th().deg();
<<<<<<< HEAD
        let mut movementDir = (idealPath - finalDir);
=======
        let mut movementDir = ((ballpos - robotpos).th() - finalDir).deg();
>>>>>>> origin/kickV2
        let ballVel = self.game.ball.velocity();
        if movementDir >= 180.0 {
            movementDir -= 360.0;
        }
        if movementDir < -180.0 {
            movementDir += 360.0;
        }
<<<<<<< HEAD
        println!("movementDir {}", movementDir);
=======

        println!("movementDir {}", movementDir );
>>>>>>> origin/kickV2

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
        if self.game.ball.height() >= 4.0 && ballVel.len() > 0.0{
            let touchPrediction = self.ballTouchPrediction();
            let mut locationByPredict = touchPrediction + (touchPrediction - *target).normalize() * (self.me.radius + self.game.ball.radius + (self.game.ball.height() - self.game.ball.radius) * 0.2) + ballVel * 0.05;
            
            self.gtp(&locationByPredict);
        } else {
            if  (movementDir.abs() < 25.0)   {
                idealPath = (*target - robotpos).th().deg();
                let sagPath = (ballpos - robotpos).th().deg();
<<<<<<< HEAD
                if ((robotCurrentPath - sagPath).abs()) < 40.0 || self.me.velocity().len() < 0.1 {
=======
                if ((robotCurrentPath - sagPath).abs()) < 15.0 && self.me.velocity().len() > 10.0{
>>>>>>> origin/kickV2
                    jump = self.rules.ROBOT_MAX_JUMP_SPEED;
                } else {
                    jump = 0.0;
                }
            } else {
                jump = 0.0;
<<<<<<< HEAD
                idealPath += shift;
            }
            self.set_robot_vel(idealPath ,100.0,jump);
=======
                idealPath = (idealPath + shift);
            }


            self.set_robot_vel(idealPath*3.1415/180.0 ,100.0,jump);
>>>>>>> origin/kickV2
        }

    }
    fn pm(&mut self, target: &Vec2) {
        self.kick(target);

    }

    fn gtp(&mut self, targetMain: & Vec2) {

        let mut target = *targetMain;
    if (target).y > self.rules.arena.depth / 2.0 {
            (target).y = self.rules.arena.depth / 2.0;
        }
        if target.y < self.rules.arena.depth / -2.0 {
            target.y = self.rules.arena.depth / -2.0;
        }
        if target.x > self.rules.arena.width / 2.0 {
            target.x = self.rules.arena.width / 2.0;
        }
        if target.x < self.rules.arena.width / -2.0 {
            target.x = self.rules.arena.width / -2.0;
        }

        let dist = self.me.position().dist(target);
        let diff = target - self.me.position();
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
            jump_speed: jump,
            use_nitro: false,
        }
    }
}
