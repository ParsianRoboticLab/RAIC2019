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
        let mut idealPath = (ballpos - robotpos).th();
        let mut movementDir = ((ballpos - robotpos).th() - finalDir)*180.0/3.1415;
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
        let robotCurrentPath = self.me.velocity().th();
        if self.game.ball.height() >= 3.5 {
            let touchPrediction = self.ballTouchPrediction();
            let locationByPredict = touchPrediction + (touchPrediction - *target).normalize() * 3.5;
            self.gtp(&locationByPredict);
        } else {
            if (robotpos.dist(ballpos) < (self.me.radius + self.game.ball.radius + 1.5)) && (movementDir.abs() < 15.0)   {
                idealPath = (*target - robotpos).th();
                let sagPath = (ballpos - robotpos).th();
                if ((robotCurrentPath - sagPath).abs() * 180.0 / 3.1415) < 40.0 || self.me.velocity().len() < 0.1 {
                    jump = self.rules.ROBOT_MAX_JUMP_SPEED;
                } else {
                    jump = 0.0;
                }
            } else {
                jump = 0.0;
                idealPath = idealPath + shift*3.1415/180.0 ;
            }
            
            
            self.set_robot_vel(idealPath ,100.0,jump);
        }

    }
    fn pm(&mut self, target: &Vec2) {
        self.kick(target);
        //self.ballTouchPrediction();
        // let ballpos = self.game.ball.position();

        // let robotpos = self.me.position() + self.me.velocity();
        // let _norm = (ballpos - *target).normalize();
        // let behind = ballpos + (ballpos - *target).normalize()*3.0;
        // let goal = ballpos - (ballpos - *target).normalize()*3.0;
        // let _avoid = ballpos + Vec2::new(4.0, 0.0);
        // if robotpos.dist(*target) < ballpos.dist(*target) - 5.0{
        //     self.gtp(&avoid);
        // }
        // else 
        // println!("dist to ball = {}", self.me.radius);
        // if robotpos.dist(behind) > 10.0 {
        //     self.gtp(&behind);
        //     println!("SALAM");
        // } else {
        //     println!("BYE");
        //     self.gtp(&goal);
        // }

    }

    fn gtp(&mut self, targetMain: & Vec2) {
        
        let mut target = *targetMain;
    if (target).y > self.rules.arena.depth / 2.0 {
            (target).y = self.rules.arena.depth / 2.0;
        }
        if(target.y < self.rules.arena.depth / -2.0) {
            target.y = self.rules.arena.depth / -2.0;
        }
        if(target.x > self.rules.arena.width / 2.0) {
            target.x = self.rules.arena.width / 2.0;
        }
        if(target.x < self.rules.arena.width / -2.0) {
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
