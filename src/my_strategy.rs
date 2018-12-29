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
include!("simulation.rs");
include!("dan.rs");
include!("circle2.rs");
include!("vec3.rs");
include!("entity3.rs");
include!("circle2.rs");
include!("dan.rs");

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
        let oppGoal = Vec2::new(0.0, self.rules.arena.depth/2.0 + 20.0);

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
    if ! v3.is_valid() {
        return Seg2::new(*vec, (seg.origin() + seg.terminal()) * 0.5)
    }
    Seg2::new(*vec, v3)
}



impl MyStrategy {


    fn gk(&mut self) {
        let y_goal = self.rules.arena.depth/-2.0 + 3.0;
        let ball_pos = self.game.ball.position();
        let goal_line = Seg2{
            origin:   Vec2{x: self.rules.arena.goal_width/2.0, y:y_goal},
            terminal: Vec2{x:self.rules.arena.goal_width/-2.0, y:y_goal}
        };
        let ball_seg = Seg2::new(self.game.ball.position(), self.game.ball.velocity()*100.0);
        let biset = get_bisect(&goal_line, &ball_pos);
        let mut target = biset.terminal();
        if self.game.ball.velocity().y < -1.0 { // KICK
            target = goal_line.intersection(ball_seg);
            if !target.is_valid() {
                target = Vec2::new(ball_pos.x, y_goal);
            }
        } else if self.game.ball.position().y  < 0.0 {
            target = Vec2{x:self.game.ball.position().x, y:y_goal};
        }
        if target.x < self.rules.arena.goal_width/-2.0 + 1.5 {
            target.x =self.rules.arena.goal_width/-2.0 + 1.5;
        } else if target.x > self.rules.arena.goal_width/2.0 - 1.5{
            target.x = self.rules.arena.goal_width/2.0 - 1.5;
        }

        self.gtp(&target);
        self.action.jump_speed = 0.0;
        if ball_pos.dist(self.me.position()) < 3.0 && self.game.ball.height() > 2.5 {
            self.action.jump_speed = self.rules.ROBOT_MAX_JUMP_SPEED;
        }

    }

    fn ballTouchPrediction(&mut self) -> Vec2 {
        let gravity = self.rules.GRAVITY;
        let ballpos = self.game.ball.position();
        let ballHeight = self.game.ball.height();
        let ballVel = self.game.ball.velocity();
        let ballhVel = self.game.ball.hVel();
        let timeToTouchTheField = (ballhVel + (ballhVel*ballhVel + 2.0*gravity*(ballHeight - self.game.ball.radius)).sqrt()) / gravity;

        let lenTravel = timeToTouchTheField * ballVel.len();
        if ballVel.len() == 0.0 {
            ballpos
        }
        else {
            ballpos + ballVel.normalize() * lenTravel
        }
    }
    fn travelTime(&mut self, target: &Vec2) -> f64 {
        let robotpos = self.me.position();
        let robotVel = self.me.velocity();
        //let mut timeR = 0.0;
        // if robotVel.len() >= self.rules.ROBOT_MAX_GROUND_SPEED{
            (*target).dist(robotpos) / robotVel.len()
        // } else {

        // }
        // 12.5
    }
    fn ballPosInTheFuture (&mut self, t : f64) -> Vec2 {
        let ballPos = self.game.ball.position();
        ballPos + self.game.ball.velocity() * t
    }

    fn kick(&mut self, target: &Vec2)  {
        let ballpos = self.game.ball.position();
        let robotpos = self.me.position();
        let robotvel = self.me.velocity();
        let finalDir = (*target - ballpos).th();
        let mut idealPath = (ballpos - robotpos).th().deg();
        let mut movementDir = ((ballpos - robotpos).th() - finalDir).deg();
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
            shift = 50.0;
        } else {
            shift = -50.0;
        }
        let mut jump = 0.0;
        let robotCurrentPath = self.me.velocity().th().deg();
        ////////
        let mut tochPoint = ballpos + (ballpos - *target).normalize()*(self.game.ball.radius - 0.5);
//kickoff
        if ballVel.len() <= 0.000000001 {
            idealPath = (tochPoint - robotpos).th().deg();
            if robotvel.len() > 25.0 {
                jump = self.game.ball.height() *4.0;
            }
            self.set_robot_vel(idealPath*3.1415/180.0 , 100.0 ,jump);
        } else {
        /////
            if self.game.ball.height() >= 6.0 {
                let touchPrediction = self.ballTouchPrediction();
                let mut locationByPredict = touchPrediction + (touchPrediction - *target).normalize() * (0.1 + self.me.radius + self.game.ball.radius + (self.game.ball.height() - self.game.ball.radius) * 0.2) + ballVel * 0.05;

                self.gtp(&locationByPredict);
            } else {

////// New prediction
                if ballVel.len() > 0.5 && movementDir.abs() < 70.0 {
                    for i in 1..100 {
                        let m = i as f64 * 0.1;
                        let bPIF = self.ballPosInTheFuture(m) + (self.ballPosInTheFuture(m) - *target).normalize()*(self.game.ball.radius - 0.5);
                        if (self.travelTime(&bPIF) - m) < 0.01 {
                            tochPoint = bPIF;
                            // while true {
                            //     println!("I'm the god {}",m);
                            // }
                            break;

                        }
                    }

                    println!("ballPos x {} y {} ,, TP x {} y {}",(ballpos + (ballpos - *target).normalize()*(self.game.ball.radius) ).x,(ballpos + (ballpos - *target).normalize()*(self.game.ball.radius)).y,tochPoint.x,tochPoint.y );
                    println!("timeball:: {}",self.travelTime(&ballpos));
                    idealPath = (tochPoint - robotpos).th().deg();
                    if ((robotCurrentPath - idealPath).abs()) < 15.0 && (self.me.velocity()-ballVel).len() > 5.0 && (tochPoint.dist(robotpos) < 5.0 && self.me.velocity().len() > 15.0) {
                        jump = 15.0;
                    } else {
                        jump = 0.0;
                    }
                }


                else if  (movementDir.abs() < 25.0)   {
                    idealPath = (tochPoint - robotpos).th().deg();
                    if ((robotCurrentPath - idealPath).abs()) < 15.0 && self.me.velocity().len() > 10.0{
                        jump = self.rules.ROBOT_MAX_JUMP_SPEED;
                    } else {
                        jump = 0.0;
                    }
                } else {
                    jump = 0.0;
                    idealPath = (idealPath + shift);
                }


                self.set_robot_vel(idealPath*DEG2RAD ,100.0,jump);
            }
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
