use crate::model::*;
use crate::strategy::Strategy;
use std::time::{Duration, Instant};

const TWO_PI : f64 = 2.0 * std::f64::consts::PI;
const EPSILON : f64 = 1.0e-6;

include!("pid.rs");
include!("vec2.rs");
include!("def.rs");
include!("draw.rs");
include!("entity.rs");
include!("angdeg.rs");
include!("seg2.rs");
include!("line2.rs");
include!("simulation.rs");
include!("dan.rs");
include!("circle2.rs");
include!("vec3.rs");
include!("entity3.rs");

pub struct MyStrategy {
    me : Robot,
    rules: Rules,
    game: Game,
    action: Action,
    height_c: usize,
    ball_future: [Ball ; 200],
    myDrawer : drawer,
    start: Instant,
    tick: i32,
}

impl Default for MyStrategy {
    fn default() -> Self {
        Self {
            me: Robot{..Default::default()},
            rules: Rules{..Default::default()},
            game: Game{..Default::default()},
            action: Action{..Default::default()},
            height_c: 0,
            ball_future: [Ball::default(); 200],
            myDrawer : drawer::default(),
            start: Instant::now(),
            tick : 0
        }
    }
}


impl Strategy for MyStrategy {

    fn act(&mut self, me: &Robot, _rules: &Rules, _game: &Game, _action: &mut Action) {
        let now = Instant::now();
        self.tick += 1;
        if !me.touch {
            println!("LOOP TIME: {:?}", now.elapsed());
            println!("TIME PASS: {:?}", self.start.elapsed());
            println!("TICK LEFT: {:?}", self.rules.max_tick_count - self.tick);
            return
        }
        // Choose Main Strategy (Coach) 1. DEF, 2. NORMAL, 3. OFF
        self.me = me.clone();
        self.rules = _rules.clone();
        self.game = _game.clone();
        self.update_ball_path();
        // Choose My Role 1. GK, 2. DEF, 3. OFF 4. SUP
        // Execute My Role
        let opp_goal = Vec2::new(0.0, self.rules.arena.depth/2.0 + 20.0);

        if me.id%2 == 1 {
            self.gk();
        } else {
            self.pm(&opp_goal);
        }


        *_action = self.action;
        println!("LOOP TIME: {:?}", now.elapsed());
        println!("TIME PASS: {:?}", self.start.elapsed());
        println!("TICK LEFT: {:?}", self.rules.max_tick_count - self.tick);
    }

    fn custom_rendering(&mut self) -> String {
        self.myDrawer.createFinalString()
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
#[derive(PartialEq)]
enum kickMode {
    chanceCreation,
    clearDanger,
    shotForGoal,
}

impl MyStrategy {

    fn update_ball_path(&mut self) {
        self.ball_future = Simulation::get_ball_path(self.me.id, &self.game, &self.rules);
        for i in 0..self.ball_future.len() {
            self.myDrawer.draw(self.ball_future[i].position3(), self.rules.BALL_RADIUS, (0.5,0.5,0.5));
        }
    }

    fn will_hit_the_ball(&self) -> bool{
        let mut me = self.me.clone();
        let mut ball = self.game.ball.clone();
        let mut action = self.action;
        let r = &self.rules;
        for _ in 0..100 {
            Self::pure_gk(&me, &ball, r, &mut action, true);
            let (col, _) = Simulation::tick(&mut me, &mut ball, &action, &self.rules);
            if col && ball.velocity().y > 0.0 && ball.position().y > -r.arena.depth/2.0 + 2.0{
                println!("COL:COL:");
                return true;
            }
        }
        false
    }

    fn pure_gk(me: &Robot, ball: &Ball, rules:&Rules, action: &mut Action, s: bool) {
        let y_goal = rules.arena.depth/-2.0 + 3.0;
        let ball_pos = ball.position();
        let goal_line = Seg2{
            origin:   Vec2{x: rules.arena.goal_width/ 2.0, y:y_goal},
            terminal: Vec2{x: rules.arena.goal_width/-2.0, y:y_goal}
        };
        let ball_seg = Seg2::new(ball.position(), ball.velocity()*100.0);
        let biset = get_bisect(&goal_line, &ball_pos);
        let mut target = biset.terminal();
        if ball.velocity().y < -1.0 { // KICK
            // if ball.position().y < -15.0 {
            //     target = ball.position();
            // } else {
            target = goal_line.intersection(ball_seg);
            // }
            if !target.is_valid() {
                target = Vec2::new(ball_pos.x, y_goal);
            }
        } else if ball.position().y  < 0.0 {
            target = Vec2{x: ball.position().x, y:y_goal};
        }
        if target.x < rules.arena.goal_width/-2.0 + 1.5 {
            target.x =rules.arena.goal_width/-2.0 + 1.5;
        } else if target.x > rules.arena.goal_width/2.0 - 1.5{
            target.x = rules.arena.goal_width/2.0 - 1.5;
        }

        Self::gtp(&target, me, rules, action);

        if s {
            action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
            action.target_velocity_y = action.jump_speed;
        } else {
            action.jump_speed = 0.0;
            action.target_velocity_y = action.jump_speed;
        }
    }

    fn gk(&mut self) {

        let clear_spot = (self.game.ball.position() - (self.game.ball.position() + self.game.ball.velocity())).normalize() * 100.0;
        let y_goal = -self.rules.arena.depth/2.0 + 3.0;
        if self.game.ball.height() < 4.0 {
            self.height_c += 1;
        } else {
            self.height_c = 0;
        }
        let ball_seg = Seg2::new(self.game.ball.position(), self.game.ball.velocity()*100.0);
        let goal_line = Seg2{
            origin:   Vec2{x: self.rules.arena.goal_width/2.0, y:y_goal},
            terminal: Vec2{x:-self.rules.arena.goal_width/2.0, y:y_goal}
        };
        // println!("LS: {}", self.height_c);
        if self.game.ball.position().y + self.game.ball.velocity().y < -20.0 {
            self.kick(&Vec2::new(0.0, 100.0), kickMode::clearDanger);
            // if self.game.ball.position().y < self.me.position().y {
            //     self.kick(&Vec2::new(0.0, -y_goal), kickMode::clearDanger);
            // } else if goal_line.intersection(ball_seg).is_valid() {
            //     self.kick(&clear_spot, kickMode::clearDanger);
            // } else {
            //     self.kick(&Vec2::new(0.0, -y_goal), kickMode::clearDanger);
            // }
        } else {
            ////
            // if self.game.ball.position().y < self.me.position().y {
            //     self.kick(&Vec2::new(0.0, -y_goal));
            // } else {
            //     Self::pure_gk(&self.me, &self.game.ball, &self.rules,&mut self.action, false);
            //     if self.will_hit_the_ball() {
            //         self.action.jump_speed = self.rules.ROBOT_MAX_JUMP_SPEED;
            //     }
            // }
            let ball_pos = self.game.ball.position();
            let goal_line = Seg2{
                origin:   Vec2{x: self.rules.arena.goal_width/2.0, y:y_goal},
                terminal: Vec2{x:-self.rules.arena.goal_width/2.0, y:y_goal}
            };
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
            if target.x < -self.rules.arena.goal_width/2.0 + 1.5 {
                target.x = -self.rules.arena.goal_width/2.0 + 1.5;
            } else if target.x > self.rules.arena.goal_width/2.0 - 1.5{
                target.x = self.rules.arena.goal_width/2.0 - 1.5;
            }

            Self::gtp(&target, &self.me, &self.rules, &mut self.action);
            self.action.jump_speed = 0.0;
            if ball_pos.dist(self.me.position()) < 3.0 && self.game.ball.height() > 2.5 {
                self.action.jump_speed = self.rules.ROBOT_MAX_JUMP_SPEED;
            }
            let ballHeight = self.game.ball.height();
            let robotCopy = self.me.clone();
            if self.if_jump_can_touch_point(&robotCopy,15.0,Vec3::new(ball_pos.x,ball_pos.y,ballHeight),Vec3::new(ball_pos.x,ball_pos.y,ballHeight),&(kickMode::clearDanger),false) {
                self.action.jump_speed = self.rules.ROBOT_MAX_JUMP_SPEED;
            }

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
    fn travel_time(&mut self, target: &Vec2) -> f64 {
        let robot_pos = self.me.position();
        let robot_vel = self.me.velocity();
        let effectiveSpeed = self.me.velocity().inner_product(&(*target - self.me.position()).normalize());
        let mut timeR = 0.0;
        if effectiveSpeed >= self.rules.ROBOT_MAX_GROUND_SPEED{
            timeR = (*target).dist(robot_pos) / effectiveSpeed;
        } else {
            let timeToVmax = (self.rules.ROBOT_MAX_GROUND_SPEED - effectiveSpeed) / self.rules.ROBOT_ACCELERATION;
            let distToVmax = 0.5 * self.rules.ROBOT_ACCELERATION * timeToVmax*timeToVmax + effectiveSpeed*timeToVmax;
            if distToVmax <= target.dist(robot_pos) {
                timeR = timeToVmax + ((*target).dist(robot_pos) - distToVmax) / self.rules.ROBOT_MAX_GROUND_SPEED;
            } else {
                timeR = (-1.0 * effectiveSpeed + (effectiveSpeed * effectiveSpeed + 2.0 * self.rules.ROBOT_ACCELERATION * (*target).dist(robot_pos)).sqrt()) / self.rules.ROBOT_MAX_GROUND_SPEED;
            }
        }
        return timeR;
        // 12.5
    }
    fn ballPosInTheFuture (&mut self, t : f64) -> Vec2 {
        let ballPos = self.game.ball.position();
        ballPos + self.game.ball.velocity() * t
    }

    fn robotJumpCalc(&mut self) {
        let maxJumpHeight = self.rules.ROBOT_MAX_JUMP_SPEED * self.rules.ROBOT_MAX_JUMP_SPEED / (2.0 * self.rules.GRAVITY);

    }
    //TODO: calc true jump speed
    fn if_jump_can_touch_point(&mut self,robot : &Robot, jump_speed : f64, _target : Vec3, _touchPoint : Vec3, kMode:& kickMode , _aggresive : bool) -> bool {
        let pos = robot.position3();
        let vel = robot.velocity3();
        let ms = self.me.max_speed();
        let mut virtualPos = pos;
        let mut virtualVel = vel;
        if virtualVel.h == 0.0 {
            virtualVel.h = jump_speed*0.9637;
        }
        let maxTime = 2.0*virtualVel.h/self.rules.GRAVITY;
        let delta_time = 0.01;
        let mut target = _target;
        if (*kMode == kickMode::clearDanger) {
            target.h -= 0.5;
            if(target.h <= 2.0) {
                target.h = 2.0;
            }
        }
        for i in 0..((100.0*maxTime) as i64) -1 {

            virtualPos += (virtualVel * delta_time);
            virtualPos.h -= (self.rules.GRAVITY*delta_time*delta_time/2.0);
            virtualVel.h -= self.rules.GRAVITY * delta_time;
            if  *kMode == kickMode::clearDanger && virtualPos.dist(target) <= self.me.radius +self.game.ball.radius - 0.5 && virtualPos.y <= target.y -0.5 && (((virtualPos.h - target.h).abs() < 0.5)  || (_aggresive == false)){
                return true;
            }
            if *kMode != kickMode::clearDanger && virtualPos.toVec2().dist(_touchPoint.toVec2()) <= 0.1 && virtualPos.dist(target) <= self.me.radius +self.game.ball.radius - 0.5 && virtualPos.y <= target.y && (((virtualPos.h - target.h).abs() < 0.5)  || (_aggresive == false)){
                return true;
            }
        }
        return false;
    }
    fn best_place_on_ball_for_kick(&mut self, finalVel : Vec3, _ball: &Ball, _radius_change_speed: f64,_rules: &Rules) -> (Vec3,f64) {
        let mut _result = Vec3::new(5000.0,5000.0,5000.0);
        let _finalVelNormm = finalVel.normalize();
        let mut _x = 0.0;
        let mut _y = 0.0;
        let mut _z = 0.0;
        let mut _theta = 0.0;
        let mut _phi:f64 = 0.0;
        let mut _impulseVec = Vec3::new(0.0,0.0,0.0);
        let _radius = _ball.radius + self.me.radius;
        let mut _outPut = Vec3::new(0.0,0.0,0.0);
        let mut bestAnswer = Vec3::new(0.0,0.0,0.0);
        let mut bestHeight = 0.0;
        let mut answerDistTheta = 1000000.0;
        let mut answerDist = 1000000.0;
        let mut bestTheta = 0.0;

        let mut robot_max_vel = 0.0;
        for i in (0..360).step_by(1) {

            _theta = (i as f64) * 180.0/3.1415;
            _phi = 90.0 * 180.0/3.1415;
            _x = _radius * (_phi.sin())*(_theta.cos()) + _ball.position3().x;
            _y = _radius * (_phi.sin())*(_theta.sin()) + _ball.position3().y;
            _z = _radius * (_phi.cos()) + _ball.position3().h;
            let mut virtualBot = &mut self.me.clone();
            virtualBot.set_position(&(Vec3::new(_x,_y,_z)));
            let virPos = virtualBot.position3();
            ///yekam sade sazio tof dare
            let tan_speed = (virtualBot.position() - self.me.position() ).normalize().inner_product(&(self.me.velocity()));
            if(tan_speed >= self.rules.ROBOT_MAX_GROUND_SPEED) {
                robot_max_vel = self.rules.ROBOT_MAX_GROUND_SPEED;
            } else {
                robot_max_vel = (2.0*self.rules.ROBOT_ACCELERATION*(*virtualBot).position().dist(self.me.position()) + tan_speed*tan_speed).sqrt();
                if (robot_max_vel >= self.rules.ROBOT_MAX_GROUND_SPEED){
                    robot_max_vel = self.rules.ROBOT_MAX_GROUND_SPEED;
                }
            }
            virtualBot.set_velocity(&((virPos - self.me.position3()).normalize()*robot_max_vel));
            _outPut = Simulation::simpleRobotBallColideStep(virtualBot,_ball,_radius_change_speed,_rules);
            let _dist = (_outPut.toVec2().normalize() - _finalVelNormm.toVec2()).len();
            if _dist < answerDistTheta {
                answerDistTheta = _dist;
                bestTheta = _theta;
                bestAnswer = Vec3::new(_x,_y,_z);
            }
        }

        let mut ball_vel_after_touch = 0.0;
        for j in (0..180).step_by(1) {
            _theta = bestTheta;
            _phi = (j as f64) * 180.0/3.1415;
            _x = _radius * (_phi.sin())*(_theta.cos()) + _ball.position3().x;
            _y = _radius * (_phi.sin())*(_theta.sin()) + _ball.position3().y;
            _z = _radius * (_phi.cos()) + _ball.position3().h;

            let mut virtualBot = &mut self.me.clone();
            let virPos = virtualBot.position3();
            // self.myDrawer.draw(Vec3::new(_x,_y,_z),0.1,(0.0,0.0,1.0));
            virtualBot.set_position(&(Vec3::new(_x,_y,_z)));
            let tan_speed = (virtualBot.position() - self.me.position() ).normalize().inner_product(&(self.me.velocity()));
            if(tan_speed >= self.rules.ROBOT_MAX_GROUND_SPEED) {
                robot_max_vel = self.rules.ROBOT_MAX_GROUND_SPEED;
            } else {
                robot_max_vel = (2.0*self.rules.ROBOT_ACCELERATION*virtualBot.position().dist(self.me.position()) + tan_speed*tan_speed).sqrt();
                if (robot_max_vel >= self.rules.ROBOT_MAX_GROUND_SPEED){
                    robot_max_vel = self.rules.ROBOT_MAX_GROUND_SPEED;
                }
            }
            virtualBot.set_velocity(&((virPos - self.me.position3()).normalize()*robot_max_vel));
            _outPut = Simulation::simpleRobotBallColideStep(virtualBot,_ball,_radius_change_speed,_rules);
            let _dist = (_outPut.normalize() - _finalVelNormm).len();
            if _dist < answerDist {
                answerDist = _dist;
                bestAnswer = Vec3::new(_x,_y,_z);
                ball_vel_after_touch = _outPut.len();
            }
        }

        return (bestAnswer,ball_vel_after_touch);
    }

    fn kick(&mut self, target: &Vec2, kMode : kickMode)  {
        let ballpos = self.game.ball.position();
        let robot_pos = self.me.position();
        let robot_vel = self.me.velocity();
        let finalDir = (*target - ballpos).th();
        let mut idealPath = (ballpos - robot_pos).th().deg();
        let mut movementDir = ((ballpos - robot_pos).th() - finalDir).normalize().deg();
        let ballVel = self.game.ball.velocity();
        let mut waitForBall = 0.0;

        //println!("movementDir {}", movementDir );

        let mut shift = 0.0;

        if movementDir.abs() < 5.0 {
            shift = 0.0;
        } else if movementDir > 0.0 {
            shift = 50.0 + 30.0/ballpos.dist(robot_pos);
        } else {
            shift = -50.0 - 30.0/ballpos.dist(robot_pos);
        }
        let mut jump = 0.0;
        let robotCurrentPath = self.me.velocity().th().deg();
        let mut new_ball = &mut self.game.ball.clone();
        let mut best_ball = &mut self.game.ball.clone();

        ////////
        let mut tochPoint = ballpos + (ballpos - *target).normalize()*(self.game.ball.radius + self.me.radius - 0.5);
        //kickoff
        if ballVel.len() <= std::f64::EPSILON  {
            // self.myDrawer.draw(self.game.ball.position3(),5.0,(1.0,0.0,0.0));
            // self.myDrawer.draw(self.me.position3(),5.0,(0.0,1.0,0.0));
            idealPath = (tochPoint - robot_pos).th().deg();
            if robot_vel.len() > 29.5  {

                jump = self.game.ball.height() *4.0;
            }
            if self.me.height() > 1.2 {
                if robot_pos.dist(ballpos) < 3.2  {
                    jump = 15.0;
                }
                else {
                    jump = 0.0;
                }
            }

            // println!("bllSpeed : {}", self.game.ball.velocity().len());
            Self::set_robot_vel(idealPath*DEG2RAD , 100.0 ,jump, &mut self.action);
        } else {
            ////// New prediction
            if movementDir.abs() < 110.0 {
                let mut ballPath = [Vec3::new(0.0,0.0,0.0) ; 600];

                let mut ballH = [0.0 ; 600];
                // let mut feasiblePoints = [Vec2::new(0.0,0.0) ; 600];
                let mut feasiblePoints = vec! [Vec2::new(0.0,0.0) ; 2];
                let mut feasiblePointsScore = vec![0.0 ; 2];
                let mut feasiblePointsHeight = vec![0.0 ; 2];
                let mut feasiblePointsTickDiff = vec![0.0 ; 2];
                let mut feasiblePointsTickNum = vec![0.0;2];
                let mut feasiblePointsMaxSpeed = vec![0.0;2];
                let mut feasiblePointsJumptTick = vec![0.0;2];
                let mut feasiblePointsJumptSpeed = vec![0.0;2];
                for j in 0..self.ball_future.len() {
                    ballPath[j] = self.ball_future[j].position();
                    ballH[j] = self.ball_future[j].height();
                    let mut bPIF = Vec2::new(0.0,0.0);
                    let mut newTarget = Vec3::new((*target).x,(*target).y,3.0);
                    let bestVec = (newTarget - ballPath[j]).normalize();
                    let rulesCopy = self.rules.clone();
                    let (best_touch_point,ball_speed_after_touch) =  self.best_place_on_ball_for_kick(bestVec,&(new_game.ball),rulesCopy.ROBOT_MAX_JUMP_SPEED,&(rulesCopy));
                    // self.myDrawer.draw(best_touch_point,1.0,(1.0,0.0,1.0));
                    // println!("best : X {} , Y {} , Z {}",best_touch_point.x,best_touch_point.y,best_touch_point.h);
                    bPIF = best_touch_point.toVec2();

                    // if kMode == kickMode::shotForGoal {
                    //     bPIF  = ballPath[j]+ (ballPath[j] - newTarget).normalize()*(self.game.ball.radius + self.me.radius - 1.0);
                    // } else if kMode == kickMode::chanceCreation {
                    //     bPIF  = ballPath[j]+ (ballPath[j] - newTarget).normalize()*(self.game.ball.radius + self.me.radius - 1.0);
                    // } else {
                    //     bPIF  = ballPath[j]+ (robot_pos - ballPath[j]).normalize()*(self.game.ball.radius - 0.5);
                    // }

                    let robottravel_time = self.travel_time(&(best_touch_point.toVec2()));
                    let mut hHeight = 6.0;
                    if kMode==kickMode::clearDanger {
                        hHeight = 6.0;
                    }

                    if robottravel_time <= (j as f64)/60.0 && (ballH[j] < hHeight) {
                        let mut max_speed = 0.0;
                        let time_needed_to_max_speed = (self.rules.ROBOT_MAX_GROUND_SPEED - self.me.velocity().len())/self.rules.ROBOT_ACCELERATION;
                        let mut exteraTime = 0.0;
                        if robottravel_time >= time_needed_to_max_speed{
                            max_speed = self.rules.ROBOT_MAX_GROUND_SPEED;
                            exteraTime = robottravel_time - time_needed_to_max_speed;
                        }
                        let mut distBeforJump = 0.0;
                        let mut found_best_sol = false;
                        for xSpeedFor in (1..(max_speed as i64) + 1).rev() {
                            for jSpeedFor in (1..(self.rules.ROBOT_MAX_JUMP_SPEED as i64) + 1).rev() {
                                let j_speed = jSpeedFor as f64;
                                let x_speed = xSpeedFor as f64;
                                if ((j_speed*j_speed)/(x_speed*x_speed) - 2.0*(self.rules.GRAVITY*ballH[j]/(x_speed*x_speed))) < 0.0 {
                                    continue;
                                }
                                let temp_distBeforJump = ((j_speed/x_speed) - ((j_speed*j_speed)/(x_speed*x_speed) - 2.0*(self.rules.GRAVITY*ballH[j]/(x_speed*x_speed))).sqrt()) / (self.rules.GRAVITY/x_speed);
                                ///// jump with maximum jump with maximum speed
                                let effectiveSpeed = self.me.velocity().inner_product(&(bPIF - self.me.position()).normalize());
                                let distNeededForThisSpeed = (x_speed*x_speed - effectiveSpeed*effectiveSpeed)/(2.0*self.rules.ROBOT_ACCELERATION);
                                // println!("best distneed {}, distBeforJump {}, sum {}",distNeededForThisSpeed,temp_distBeforJump,temp_distBeforJump + distNeededForThisSpeed);

                                // if distNeededForThisSpeed < 0.0 {
                                //     continue;
                                // }
                                if temp_distBeforJump + distNeededForThisSpeed <= robot_pos.dist(ballPath[j].toVec2()){
                                    feasiblePointsMaxSpeed.push(0.0);
                                    feasiblePoints.push(bPIF);
                                    let mut _point = 4000.0 / (j as f64) ;
                                    if ballPath[j].x.abs() <= self.rules.arena.goal_width/2.0 - 1.0 && kMode!=kickMode::clearDanger {
                                        _point += ball_speed_after_touch;
                                        if ballPath[j].y >= 0.0 {
                                            _point = _point + 25.0 + ballPath[j].y ;
                                        }
                                    }
                                    // if robot_theta.abs() <= theta_app {
                                    //     _point += 30.0;
                                    // }
                                    feasiblePointsScore.push(_point);
                                    feasiblePointsHeight.push (ballH[j]);
                                    //println!("tick needed: {} , time for walk {}, time for jump {}", j, self.travel_time(&(bPIF - (robot_pos - bPIF).normalize()*temp_distBeforJump)) , (temp_distBeforJump/x_speed));
                                    feasiblePointsTickDiff.push((j as f64)/60.0 - self.travel_time(&(bPIF - (robot_pos - bPIF).normalize()*temp_distBeforJump)) + (temp_distBeforJump/x_speed));
                                    feasiblePointsTickNum.push(j as f64);
                                    feasiblePointsJumptSpeed.push(j_speed);
                                    found_best_sol = true;
                                    break;
                                }

                            }
                            if found_best_sol == true {
                                break;
                            }
                        }


                    }

                }
                // println!("number of feasiblePoints {}",feasiblePoints.len() );
                let mut biggestPoint = 0.0;
                let mut bestHeight = 0.0;
                let mut bestTick = 0;
                let mut bestJumpSpeed = 0.0;
                for i in 0..feasiblePoints.len()-1 {
                    if feasiblePointsScore[i] >= biggestPoint {
                        biggestPoint = feasiblePointsScore[i];
                        tochPoint = feasiblePoints[i];
                        waitForBall = feasiblePointsTickDiff[i];
                        bestHeight = feasiblePointsHeight[i] ;
                        bestTick = feasiblePointsTickNum[i] as usize;
                        bestJumpSpeed = feasiblePointsJumptSpeed[i];
                    }
                }

                let bestBallPos = ballPath[bestTick].toVec2();
                self.myDrawer.draw(Vec3::new(bestBallPos.x,bestBallPos.y,bestHeight),2.0,(1.0,0.0,0.0));
                self.myDrawer.draw(Vec3::new(tochPoint.x,tochPoint.y,bestHeight),1.0,(0.0,1.0,0.0));
                println!("best Height : {} , x {}, y {}" , bestHeight,tochPoint.x,tochPoint.y);
                let robot_theta_move = ((tochPoint - robot_pos).th() - (*target - tochPoint).th()).deg();

                let mut tuchPFJ = tochPoint;
                // if (kMode == kickMode::clearDanger) {
                //     tuchPFJ = bestBallPos;
                // }
                let mut findAgg = false;
                let copyMe = self.me.clone();
                println!("best JS : {}",bestJumpSpeed);
                for jSpeed in ((bestJumpSpeed as i64)..16).rev() {

                    if self.if_jump_can_touch_point(&copyMe,jSpeed as f64,Vec3::new(tuchPFJ.x,tuchPFJ.y,bestHeight),Vec3::new(tochPoint.x,tochPoint.y,bestHeight),&(kMode),true) && waitForBall <= 0.05 {
                        jump = (jSpeed as f64);
                        findAgg = true;
                        break;
                    } else {
                        jump = 0.0;
                    }
                }
                if findAgg == false && kMode !=kickMode::shotForGoal{
                    for jSpeed in ((bestJumpSpeed as i64)..(bestJumpSpeed as i64)+1).rev() {
                        if self.if_jump_can_touch_point(&copyMe.clone(),jSpeed as f64,Vec3::new(tuchPFJ.x,tuchPFJ.y,bestHeight),Vec3::new(tochPoint.x,tochPoint.y,bestHeight),&(kMode),false) && waitForBall <= 0.05 {
                            jump = jSpeed as f64;
                            findAgg = true;
                            break;
                        } else {
                            jump = 0.0;
                        }
                    }
                }
                // if (self.me.height() > self.me.radius + 0.1) && self.me.position3().dist(self.game.ball.position3()) >= (self.me.radius + self.game.ball.radius) + 0.1 {
                //     jump = 0.0;
                // }

                if waitForBall > 0.1  {//}&& tochPoint.y >= 10.0{ // && tochPoint.y > robot_pos.y &&p robot_pos.y > 0.0{
                    let maxSpeedDist = self.rules.ROBOT_MAX_GROUND_SPEED*self.rules.ROBOT_MAX_GROUND_SPEED / self.rules.ROBOT_ACCELERATION ;
                    if ballpos.y >= 10.0 {
                    tochPoint = Vec2::new(0.0,0.0);
                    }//tochPoint + (tochPoint - *target).normalize() * (maxSpeedDist/2.0);
                    //let kickSeg = Seg2::new(tochPoint + (tochPoint - *target).normalize()*(self.game.ball.height()/10.0 + 5.5),tochPoint + (tochPoint - *target).normalize()*15.0);

                    //tochPoint = kickSeg.nearest_point(&robot_pos);

                    if(tochPoint.y <= self.rules.arena.depth / -2.0 - 5.0 )
                    {
                        tochPoint.y = self.rules.arena.depth / -2.0 - 5.0;
                    }

                }
                if tochPoint.x > self.rules.arena.width / 2.0 - 2.0{
                    tochPoint.x = self.rules.arena.width / 2.0 - 2.0;
                }
                if tochPoint.x < self.rules.arena.width / -2.0  + 2.0{
                    tochPoint.x = self.rules.arena.width / -2.0 + 2.0;
                }
                idealPath = (tochPoint - robot_pos).th().deg();

            }
            else {
                jump = 0.0;
                idealPath = (idealPath + shift);
            }
            ////println!("jump {},, result {}, nesbat {}",jump,self.me.velocity3().h, self.me.velocity3().h / jump);
            if waitForBall >= 0.1 && ballpos.y < 10.0{
                Self::set_robot_vel(idealPath*DEG2RAD ,0.0,jump, &mut self.action);
        } else {
            Self::set_robot_vel(idealPath*DEG2RAD ,100.0,jump, &mut self.action);
        }
        }
    }
    fn pm(&mut self, target: &Vec2) {
        let mut newTarget = *target;
        newTarget.x -= self.game.ball.velocity().x * 1.2;
        if self.game.ball.position().x > self.rules.arena.width / 2.0 - 1.0 {
            newTarget.x = self.rules.arena.width / 2.0 - 1.0;
        }
        if self.game.ball.position().x < self.rules.arena.width / -2.0  + 1.0 {
            newTarget.x = self.rules.arena.width / -2.0 +1.0;
        }


        self.kick(&newTarget,kickMode::shotForGoal);

    }

    fn gtp(target_main: &Vec2, me: &Robot, _rules: &Rules, action: &mut Action) {

        let mut target = *target_main;
        if (target).y > _rules.arena.depth / 2.0 {
            (target).y = _rules.arena.depth / 2.0;
        }
        if target.y < _rules.arena.depth / -2.0 {
            target.y = _rules.arena.depth / -2.0;
        }
        if target.x > _rules.arena.width / 2.0 {
            target.x = _rules.arena.width / 2.0;
        }
        if target.x < _rules.arena.width / -2.0 {
            target.x = _rules.arena.width / -2.0;
        }

        let dist = me.position().dist(target);
        let diff = target - me.position();
        let angle = (diff.y).atan2(diff.x);
        Self::set_robot_vel(angle, 5.0 * dist , 0.0, action);


    }

    fn set_robot_vel(angle : f64, vel: f64, jump : f64, action: &mut Action) {
        *action = Action {
            target_velocity_x: vel*angle.cos(),
            target_velocity_y: 15.0,
            target_velocity_z: vel*angle.sin(),
            jump_speed: jump,
            use_nitro: false,
        }
    }
}
