use crate::model::*;
use crate::strategy::Strategy;
use std::time::{Duration, Instant};

const TWO_PI : f64 = 2.0 * std::f64::consts::PI;
const EPSILON : f64 = 1.0e-6;
const BALL_PREDICTION_TICKS : usize = 160;
const CAN_DRAW : bool = false;
const global_step_time : f64 = 1.0/ ((60 as f64));

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
    robot_hist : Vec<Robot>,
    me : Robot,
    rules: Rules,
    game: Game,
    action: Action,
    height_c: usize,
    ball_future: [Ball ; BALL_PREDICTION_TICKS],
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
            robot_hist : vec![Robot::default() ; 1],
            ball_future: [Ball::default(); BALL_PREDICTION_TICKS],
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
        // if !me.touch {
        //     println!("LOOP TIME: {:?}", now.elapsed());
        //     println!("TIME PASS: {:?}", self.start.elapsed());
        //     println!("TICK LEFT: {:?}", self.rules.max_tick_count - self.tick);
        //     return
        // }
        // Choose Main Strategy (Coach) 1. DEF, 2. NORMAL, 3. OFF
        self.me = me.clone();
        self.rules = _rules.clone();
        self.game = _game.clone();
        self.update_ball_path();
        // Choose My Role 1. GK, 2. DEF, 3. OFF 4. SUP
        // Execute My Role
        let opp_goal = Vec2::new(0.0, self.rules.arena.depth/2.0 + 2.0);

        if me.id%2 == 1 {
            self.gk();
        } else {
            self.pm(&opp_goal);
        }


        *_action = self.action;
        println!("LOOP TIME: {:?}", now.elapsed());
        println!("TIME PASS: {:?}", self.start.elapsed());
        println!("TICK LEFT: {:?}", self.rules.max_tick_count - self.tick);
        let meCopy = self.me.clone();
        self.robot_hist.insert(0,meCopy);
        self.robot_hist.truncate(100);
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

#[derive(PartialEq,Copy, Clone, Debug)]
enum kickMode {
    chanceCreation,
    clearDanger,
    shotForGoal,
}

impl MyStrategy {

    fn update_ball_path(&mut self) {
        self.ball_future = Simulation::get_ball_path(self.me.id, &self.game, &self.rules);
        // for i in 0..self.ball_future.len() {
        //     self.myDrawer.draw(self.ball_future[i].position3(), self.rules.BALL_RADIUS, (0.5,0.5,0.5));
        // }
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
        let y_goal = rules.arena.depth/-2.0 + 1.0;
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

        let clear_spot = (self.game.ball.position() - self.me.position()).normalize() * 100.0;
        let y_goal = -self.rules.arena.depth/2.0 + 1.0;
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
        if self.game.ball.position().y  + self.game.ball.velocity().y < -20.0 {
            //     self.kick(&Vec2::new(0.0, 100.0), kickMode::clearDanger);
            if self.game.ball.position().y < self.me.position().y {
                self.kick(&Vec2::new(0.0, -y_goal), kickMode::clearDanger);
            } else  {
                self.kick(&clear_spot, kickMode::clearDanger);
            }
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
            // let goal_line = Seg2{
            //     origin:   Vec2{x: self.rules.arena.goal_width/2.0, y:y_goal},
            //     terminal: Vec2{x:-self.rules.arena.goal_width/2.0, y:y_goal}
            // };
            // let biset = get_bisect(&goal_line, &ball_pos);
            let mut target = Vec2::new(0.0,y_goal);
            // if self.game.ball.velocity().y < -1.0 { // KICK
            //     target = goal_line.intersection(ball_seg);
            //     if !target.is_valid() {
            //         target = Vec2::new(ball_pos.x, y_goal);
            //     }
            // } else if self.game.ball.position().y  < 0.0 {
            //     target = Vec2{x:self.game.ball.position().x, y:y_goal};
            // }
            // if target.x < -self.rules.arena.goal_width/2.0 + 1.5 {
            //     target.x = -self.rules.arena.goal_width/2.0 + 1.5;
            // } else if target.x > self.rules.arena.goal_width/2.0 - 1.5{
            //     target.x = self.rules.arena.goal_width/2.0 - 1.5;
            // }


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
        let effectiveSpeed = self.me.velocity().inner_product(&((*target - self.me.position()).normalize()));
        let mut timeR = 0.0;
        if effectiveSpeed >= self.rules.ROBOT_MAX_GROUND_SPEED{
            timeR = (*target).dist(robot_pos) / effectiveSpeed;
        } else {
            let timeToVmax = (self.rules.ROBOT_MAX_GROUND_SPEED - effectiveSpeed) / self.rules.ROBOT_ACCELERATION;
            let distToVmax = 0.5 * self.rules.ROBOT_ACCELERATION * timeToVmax*timeToVmax + effectiveSpeed*timeToVmax;
            if distToVmax <= target.dist(robot_pos) {
                timeR = timeToVmax + ((*target).dist(robot_pos) - distToVmax) / self.rules.ROBOT_MAX_GROUND_SPEED;
            } else {
                timeR = (-1.0 * effectiveSpeed + (effectiveSpeed * effectiveSpeed + 2.0 * self.rules.ROBOT_ACCELERATION * (*target).dist(robot_pos)).sqrt()) / self.rules.ROBOT_ACCELERATION;
            }
        }
        return timeR;
        // 12.5
    }
    fn travel_time_alt(&mut self, _mePos : Vec2, _meVel: Vec2,target: &Vec2) -> f64 {
        let robot_pos = _mePos;
        let robot_vel = _meVel;
        let effectiveSpeed = robot_vel.inner_product(&((*target - robot_pos).normalize()));
        let mut timeR = 0.0;
        if effectiveSpeed >= self.rules.ROBOT_MAX_GROUND_SPEED{
            timeR = (*target).dist(robot_pos) / effectiveSpeed;
        } else {
            let timeToVmax = (self.rules.ROBOT_MAX_GROUND_SPEED - effectiveSpeed) / self.rules.ROBOT_ACCELERATION;
            let distToVmax = 0.5 * self.rules.ROBOT_ACCELERATION * timeToVmax*timeToVmax + effectiveSpeed*timeToVmax;
            if distToVmax <= target.dist(robot_pos) {
                timeR = timeToVmax + ((*target).dist(robot_pos) - distToVmax) / self.rules.ROBOT_MAX_GROUND_SPEED;
            } else {
                timeR = (-1.0 * effectiveSpeed + (effectiveSpeed * effectiveSpeed + 2.0 * self.rules.ROBOT_ACCELERATION * (*target).dist(robot_pos)).sqrt()) / self.rules.ROBOT_ACCELERATION;
            }
        }
        return timeR;
        // 12.5
    }

    fn travel_time_it(&mut self, target: &Vec2) -> f64 {
        let mut velocity = self.me.velocity();
        let mut position = self.me.position();
        let mut result = 10000.0;
        let perS = 5.0;
        let step_time = (1.0/ ((self.rules.TICKS_PER_SECOND as f64)*perS));
        for i in 0..BALL_PREDICTION_TICKS*5 {
            let idealPath = (*target - position).th().deg()*DEG2RAD;
            if position.dist(*target) <= 0.05{
                result = (i as f64) * step_time;
                break;
            }
            position = position + velocity * step_time;
            let target_vel_change = (*target - position).normalize()*self.rules.ROBOT_MAX_GROUND_SPEED - velocity;
            if target_vel_change.len() > 0.0 {
                let acc = self.rules.ROBOT_ACCELERATION;// * _me.touch_normal_y.unwrap().max(0.0);

                velocity = velocity +target_vel_change.normalize() * acc * step_time;

                if velocity.len() > self.rules.ROBOT_MAX_GROUND_SPEED {
                    velocity = velocity.normalize() *self.rules.ROBOT_MAX_GROUND_SPEED;
                }
            }

        }
        let oldTime = self.travel_time(target);
        if result - oldTime >= 1.0 {
            result = oldTime;
        }
        return result;

    }

    fn travel_time_it_test(&mut self, target: &Vec2) -> f64 {
        let mut velocity = self.me.velocity();
        let mut position = self.me.position();
        let mut result = 10000.0;
        let perS = 1.0;
        let step_time = (1.0/ ((self.rules.TICKS_PER_SECOND as f64)*perS));
        for i in 0..BALL_PREDICTION_TICKS*1 {
            let idealPath = (*target - position).th().deg()*DEG2RAD;
            if position.dist(*target) <= 0.1{
                result = (i as f64) * step_time;
                break;
            }
            self.myDrawer.draw(Vec3::new(position.x,position.y,1.0),0.5,(1.0,1.0,1.0));
            position = position + velocity * step_time;
            let target_vel_change = (*target - position).normalize()*self.rules.ROBOT_MAX_GROUND_SPEED - velocity;
            if target_vel_change.len() > 0.0 {
                let acc = self.rules.ROBOT_ACCELERATION;// * _me.touch_normal_y.unwrap().max(0.0);
                velocity = velocity +target_vel_change.normalize() * acc * step_time;
                if velocity.len() > self.rules.ROBOT_MAX_GROUND_SPEED {
                    velocity = velocity.normalize() *self.rules.ROBOT_MAX_GROUND_SPEED;
                }
            }

        }
        let oldTime = self.travel_time(target);
        if result - oldTime >= 1.0 {
            result = oldTime;
        }
        return result;

    }

    fn god_step(&mut self,_position : Vec3 , _velocity : Vec3 , target : Vec2 , _j_speed : f64 ) -> (Vec3,Vec3) {
        let mut velocity = _velocity;
        let mut position = _position;
        let step_time = (1.0/ ((self.rules.TICKS_PER_SECOND as f64)));

        position = position + velocity * step_time;
        position.h = (position.h - (self.rules.GRAVITY * step_time * step_time / 2.0));
        velocity.h = velocity.h - (self.rules.GRAVITY * step_time);
        if position.h < self.me.radius {
            position.h = self.me.radius;
            velocity.h = 0.0;
        }


        if position.h <= self.me.radius {
            let target_vel_change = (target - position.toVec2()).normalize()*self.rules.ROBOT_MAX_GROUND_SPEED - velocity.toVec2();
            if target_vel_change.len() > 0.0 {
                let acc = self.rules.ROBOT_ACCELERATION;// * _me.touch_normal_y.unwrap().max(0.0);
                velocity.x = velocity.x + (target_vel_change.normalize() * acc * step_time).x;
                velocity.y = velocity.y + (target_vel_change.normalize() * acc * step_time).y;

                if velocity.toVec2().len() > self.rules.ROBOT_MAX_GROUND_SPEED {
                    velocity = Vec3::new((velocity.toVec2().normalize() *self.rules.ROBOT_MAX_GROUND_SPEED).x,(velocity.toVec2().normalize() *self.rules.ROBOT_MAX_GROUND_SPEED).y,velocity.h);
                }
            }

            if _j_speed > 0.0 {
                velocity.h = _j_speed;
            }
        }

        return (position,velocity);

    }
    fn god_step_GP(&mut self,_position : Vec3 , _velocity : Vec3 , _target : Vec2 , _j_speed : f64 ) -> (Vec3,Vec3) {
        let mut velocity = _velocity;
        let mut position = _position;
        let step_time = (1.0/ ((self.rules.TICKS_PER_SECOND as f64)));
        let mut target = _target;
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

        let dist = _position.toVec2().dist(target);
        let _ac_speed = dist * 3.0;

        position = position + velocity * step_time;
        position.h = (position.h - (self.rules.GRAVITY * step_time * step_time / 2.0));
        velocity.h = velocity.h - (self.rules.GRAVITY * step_time);
        if position.h < self.me.radius {
            position.h = self.me.radius;
            velocity.h = 0.0;
        }


        if position.h <= self.me.radius {
            let target_vel_change = (target - position.toVec2()).normalize()*_ac_speed - velocity.toVec2();
            if target_vel_change.len() > 0.0 {
                let acc = self.rules.ROBOT_ACCELERATION;// * _me.touch_normal_y.unwrap().max(0.0);
                velocity.x = velocity.x + (target_vel_change.normalize() * acc * step_time).x;
                velocity.y = velocity.y + (target_vel_change.normalize() * acc * step_time).y;

                if velocity.toVec2().len() > _ac_speed{
                    velocity = Vec3::new((velocity.toVec2().normalize() *_ac_speed).x,(velocity.toVec2().normalize() *_ac_speed).y,velocity.h);
                }
            }

            if _j_speed > 0.0 {
                velocity.h = _j_speed;
            }
        }

        return (position,velocity);

    }

    fn god_simulation (&mut self , _ball : Ball,_touch_point : Vec3,_target : Vec3, _time_availabe : f64, _kMode : kickMode) -> (bool,Vec3,f64,f64,f64,f64){
        let step_time = 1.0/ ((self.rules.TICKS_PER_SECOND as f64));
        let mut position = self.me.position3();
        let mut vel = self.me.velocity3();
        let ballPos = _ball.position3();
        let ballVel = _ball.velocity3();
        let tochPoint = _touch_point.toVec2();
        let mut result = (false,Vec3::new(0.0,0.0,0.0),0.0,0.0,0.0,0.0);
        let target = _target.toVec2();
        let mut found_best_sol = false;
        let mut _vir_robot_target = Vec2::new(0.0,0.0);
        let mut jump_tick_time = 0.0;
        let mut best_jump_speed = 0.0;
        let mut best_speed = 0.0;
        let mut t_change_dir = 0.0;
        let mut dist_before_jump = 0.0;
        let mut jump_time_output = 10000.0;
        _vir_robot_target = tochPoint;
        let mut wasteTime = 1000.0;
        let mut canJump = 1000.0;
        let mut selectedPath = [Vec3::new(0.0,0.0,0.0) ;600];
        let _time_to_reach_for_wait = self.travel_time_alt(position.toVec2(), vel.toVec2(), &(tochPoint));
        if (_time_to_reach_for_wait >= _time_availabe - step_time) {
            wasteTime = 0.0;
        }
        self.myDrawer.drawText(format!("bekhoda khorde , reach_time {}, ball_time {}",_time_to_reach_for_wait,_time_availabe));

        for i in 0..(((_time_availabe)*(self.rules.TICKS_PER_SECOND as f64)) as usize + 40) {
            selectedPath[i] = position;
            let _time = (i as f64) / (self.rules.TICKS_PER_SECOND as f64);
            let mut _time_to_reach = self.travel_time_alt(position.toVec2(), vel.toVec2(), &(tochPoint));
            if _time_to_reach + _time  < _time_availabe - step_time {
                let maxSpeedDist = self.rules.ROBOT_MAX_GROUND_SPEED*self.rules.ROBOT_MAX_GROUND_SPEED / self.rules.ROBOT_ACCELERATION ;
                let kickSeg = Seg2::new(tochPoint + (tochPoint - target).normalize()*(maxSpeedDist / 2.0),tochPoint + (tochPoint - target).normalize()*15.0);
                let mut altPoint = tochPoint;
                altPoint.y =  tochPoint.y- maxSpeedDist ;//kickSeg.nearest_point(&(position.toVec2()));
                if(_kMode == kickMode::clearDanger) {
                    let y_goal = self.rules.arena.depth/-2.0 + 1.0;
                    altPoint = Vec2::new(0.0,y_goal);
                }
                let stepRes = self.god_step_GP(position, vel, altPoint, 0.0);
                position = stepRes.0;
                vel = stepRes.1;
            } else {
                for jSpeedFor in (0..(self.rules.ROBOT_MAX_JUMP_SPEED as i64) + 1).rev() {
                    if found_best_sol == true {
                        break;
                    }
                    for xSpeedFor in (0..(self.rules.ROBOT_MAX_GROUND_SPEED as i64) + 1).rev() {
                        let j_speed = jSpeedFor as f64;
                        let mut x_speed = xSpeedFor as f64;
                        if x_speed == 0.0 {
                            x_speed = 0.000001;
                        }
                        if ((j_speed*j_speed)/(x_speed*x_speed) - 2.0*((self.rules.GRAVITY*(_touch_point.h - self.me.radius))/(x_speed*x_speed))) < 0.0 {
                            continue;
                        }
                        let temp_distBeforJumpDown = ((j_speed/x_speed) + ((j_speed*j_speed)/(x_speed*x_speed) - 2.0*((self.rules.GRAVITY*(_touch_point.h - self.me.radius))/(x_speed*x_speed))).sqrt()) / (self.rules.GRAVITY/(x_speed*x_speed));
                        let jumptimeDown = temp_distBeforJumpDown/x_speed;
                        ///// jump with maximum jump with maximum speed
                        let effectiveSpeed = vel.toVec2().inner_product(&(tochPoint - position.toVec2()).normalize());
                        let distNeededForThisSpeed = (x_speed*x_speed - effectiveSpeed*effectiveSpeed)/(2.0*self.rules.ROBOT_ACCELERATION);

                        let temp_distBeforJumpUP = ((j_speed/x_speed) - ((j_speed*j_speed)/(x_speed*x_speed) - 2.0*((self.rules.GRAVITY*(_touch_point.h - self.me.radius))/(x_speed*x_speed))).sqrt()) / (self.rules.GRAVITY/(x_speed*x_speed));
                        let jumptimeUP = temp_distBeforJumpUP/x_speed;
                        let mut canJump = 0;
                        let mut temp_distBeforJump = 10000.0;
                        let mut jump_time = 10000000.0;


                        if temp_distBeforJumpUP + distNeededForThisSpeed <= position.toVec2().dist(tochPoint) {
                            temp_distBeforJump = temp_distBeforJumpUP;
                            jump_time= jumptimeUP;
                        }
                        else if temp_distBeforJumpDown + distNeededForThisSpeed <= position.toVec2().dist(tochPoint){
                            temp_distBeforJump = temp_distBeforJumpDown;
                            jump_time= jumptimeDown;
                        }

                        if temp_distBeforJump + distNeededForThisSpeed <= position.toVec2().dist(tochPoint){
                            _vir_robot_target = tochPoint;
                            jump_tick_time = _time_availabe - jump_time;
                            jump_time_output = jump_time;
                            best_jump_speed = j_speed;
                            found_best_sol = true;
                            best_speed = x_speed;
                            dist_before_jump = temp_distBeforJump;
                            break;
                        }
                    }

                }
                let mut _j_for_kick = 0.0;
                if _time >= jump_tick_time  {
                    _j_for_kick = best_jump_speed;
                }
                if jump_tick_time < step_time {
                    canJump = 0.0;
                }
                // println!("best JS {}" , _j_for_kick);
                let stepRes = self.god_step(position, vel, _vir_robot_target, _j_for_kick);
                position = stepRes.0;
                vel = stepRes.1;
            }
            let mut virtualBot = self.me.clone();
            virtualBot.set_position(&position);
            virtualBot.set_velocity(&vel);

            if position.dist(_ball.position3()) < self.me.radius + self.game.ball.radius  {
                for j in 0..(((_time_availabe)*(self.rules.TICKS_PER_SECOND as f64)) as usize + 10) {
                    self.myDrawer.draw(selectedPath[j],0.5,(1.0,0.0,0.0));
                }

                result.5 = canJump;
                result.4 = wasteTime;
                result.3 = best_speed;
                result.2 = best_jump_speed;
                result.1 = Simulation::simpleRobotBallColideStep(&virtualBot,&_ball,best_jump_speed,&self.rules);
                result.0 = true;
                break;
            }

        }

        return result;
    }
    fn calc_point_for_reflect_kick(&mut self,_ball : Ball , _target : Vec3) -> Vec3{
        let ball2D = _ball.position();
        let target2D = _target.toVec2();
        let mut x = self.rules.arena.width / 2.0;
        let mut result = _target;
        if ball2D.x < 0.0 {
            x = -1.0*x;
        }
        result.x = x;
        result.y = (ball2D.y*x - ball2D.y*target2D.x + target2D.y*x - target2D.y * ball2D.x) / (2.0*x - ball2D.x - target2D.x);
        return result;

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

        for i in 0..((100.0*maxTime) as i64) -1 {

            virtualPos += (virtualVel * delta_time);
            virtualPos.h -= (self.rules.GRAVITY*delta_time*delta_time/2.0);
            virtualVel.h -= self.rules.GRAVITY * delta_time;
            if  virtualPos.dist(target) <= self.me.radius +self.game.ball.radius - 0.2{
                return true;
            }
        }
        return false;
    }
    fn best_place_on_ball_for_kick(&mut self, finalVel : Vec3, _ball: &Ball, _radius_change_speed: f64,_rules: &Rules, _k_mode : kickMode) -> (Vec3,f64) {
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

        let mut ball_vel_after_touch = 0.0;
        let kick_path_theta = (finalVel.toVec2().th().deg() as i64);
        // self.myDrawer.drawText(format!("ang : {} ",angForBestKick*RAD2DEG));
        bestAnswer.h = _ball.position3().h - finalVel.h;


        if (true || _k_mode == kickMode::clearDanger) {
            bestAnswer.h = _ball.position3().h - self.me.radius ;
        }
        if bestAnswer.h < self.me.radius {
            bestAnswer.h = self.me.radius;
        }
        if bestAnswer.h > 7.75 {
            bestAnswer.h = 7.75;
        }

        let angForBestKick = ((bestAnswer.h - _ball.position3().h) / finalVel.toVec2().len()).atan() + 3.1415/2.0;

        for i in ((90+kick_path_theta)..(270+kick_path_theta)).step_by(5) {

            _theta = (i as f64) * 180.0/3.1415;
            _phi = -angForBestKick;// 90.0 * 180.0/3.1415;
            _x = _radius * (_phi.sin())*(_theta.cos()) + _ball.position3().x;
            _y = _radius * (_phi.sin())*(_theta.sin()) + _ball.position3().y;
            _z = bestAnswer.h;
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
                ball_vel_after_touch = _outPut.toVec2().inner_product(&(finalVel.toVec2().normalize()));
            }
        }

        // for j in (0..180).step_by(1) {
        //     _theta = bestTheta;
        //     _phi = (j as f64) * 180.0/3.1415;
        //     _x = _radius * (_phi.sin())*(_theta.cos()) + _ball.position3().x;
        //     _y = _radius * (_phi.sin())*(_theta.sin()) + _ball.position3().y;
        //     _z = _radius * (_phi.cos()) + _ball.position3().h;
        //
        //     let mut virtualBot = &mut self.me.clone();
        //     let virPos = virtualBot.position3();
        //     // self.myDrawer.draw(Vec3::new(_x,_y,_z),0.1,(0.0,0.0,1.0));
        //     virtualBot.set_position(&(Vec3::new(_x,_y,_z)));
        //     let tan_speed = (virtualBot.position() - self.me.position() ).normalize().inner_product(&(self.me.velocity()));
        //     if(tan_speed >= self.rules.ROBOT_MAX_GROUND_SPEED) {
        //         robot_max_vel = self.rules.ROBOT_MAX_GROUND_SPEED;
        //     } else {
        //         robot_max_vel = (2.0*self.rules.ROBOT_ACCELERATION*virtualBot.position().dist(self.me.position()) + tan_speed*tan_speed).sqrt();
        //         if (robot_max_vel >= self.rules.ROBOT_MAX_GROUND_SPEED){
        //             robot_max_vel = self.rules.ROBOT_MAX_GROUND_SPEED;
        //         }
        //     }
        //     virtualBot.set_velocity(&((virPos - self.me.position3()).normalize()*robot_max_vel));
        //     _outPut = Simulation::simpleRobotBallColideStep(virtualBot,_ball,_radius_change_speed,_rules);
        //     let _dist = (_outPut.normalize() - _finalVelNormm).len();
        //     if _dist < answerDist {
        //         answerDist = _dist;
        //         bestAnswer = Vec3::new(_x,_y,_z);
        //         ball_vel_after_touch = _outPut.len();
        //     }
        // }

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
        let mut biggestPoint = 0.0;
        let mut bestHeight = 0.0;
        let mut bestTick = 0;
        let mut bestJumpSpeed = 0.0;
        let mut best_tick_beforeJump = 0.0;
        if ballVel.len() <= std::f64::EPSILON  {
            // self.myDrawer.draw(self.game.ball.position3(),5.0,(1.0,0.0,0.0));
            // self.myDrawer.draw(self.me.position3(),5.0,(0.0,1.0,0.0));

            idealPath = (tochPoint - robot_pos).th().deg();
            // let sagR = self.travel_time_it(&tochPoint);
            // self.myDrawer.drawText(format!("{}",sagR));

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
            if movementDir.abs() < 110.0 || kMode == kickMode::shotForGoal{
                let mut ballPath = [Vec3::new(0.0,0.0,0.0) ; 600];
                let mut wtfIsThis = false;
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
                let mut feasiblePointsTickBJ = vec![0.0 ;2];

                for j in 0..self.ball_future.len() {
                    ballPath[j] = self.ball_future[j].position3();
                    ballH[j] = self.ball_future[j].height();
                    let BC = self.ball_future[j].clone();

                    let mut bPIF = Vec2::new(0.0,0.0);
                    let mut newTarget = Vec3::new((*target).x,(*target).y ,7.0);
                    let reflect_target = self.calc_point_for_reflect_kick(BC, newTarget);
                    if (ballPath[j].toVec2() - robot_pos).normalize().inner_product(&(newTarget.toVec2() - ballPath[j].toVec2()).normalize()) <
                    (ballPath[j].toVec2() - robot_pos).normalize().inner_product(&(reflect_target.toVec2() - ballPath[j].toVec2()).normalize())
                     && kMode==kickMode::shotForGoal && ballPath[j].y < self.rules.arena.depth / 2.0 - 20.0{
                            newTarget = reflect_target;
                    }

                    let bestVec = (newTarget - ballPath[j]).normalize();

                    let rulesCopy = self.rules.clone();
                    let virBall = self.ball_future[j];
                    // self.myDrawer.draw(newTarget,0.5,(1.0,1.0,0.5));
                    let (best_touch_point,ball_speed_after_touch) =  self.best_place_on_ball_for_kick(bestVec,&(virBall),rulesCopy.ROBOT_MAX_JUMP_SPEED,&(rulesCopy),kMode);
                    bPIF = best_touch_point.toVec2();

                    let robottravel_time = self.travel_time(&(best_touch_point.toVec2()));
                    let mut hHeight = 8.0;
                    if kMode==kickMode::clearDanger {
                        hHeight = 8.0;
                    }


                    if (robottravel_time -(j as f64)/(self.rules.TICKS_PER_SECOND as f64)) <= global_step_time  && (ballH[j] < hHeight) {
                        self.myDrawer.draw(best_touch_point,0.5,(1.0,1.0,1.0));
                        self.myDrawer.draw(BC.position3(),2.0,(0.0,1.0,0.0));

                        //self.travel_time_it_test(&bPIF);

                        let mut max_speed = 0.0;
                        let time_needed_to_max_speed = (self.rules.ROBOT_MAX_GROUND_SPEED - self.me.velocity().len())/self.rules.ROBOT_ACCELERATION;
                        let mut exteraTime = 0.0;
                        if robottravel_time >= time_needed_to_max_speed{
                            max_speed = self.rules.ROBOT_MAX_GROUND_SPEED;
                            exteraTime = robottravel_time - time_needed_to_max_speed;
                        }
                        let mut distBeforJump = 0.0;
                        let mut found_best_sol = false;
                        //////// some tofe khub

                        if false {//}((ballH[j] <= 2.8002) || bPIF.dist(robot_pos) <= 0.3) && bPIF.y >= robot_pos.y {
                            feasiblePointsMaxSpeed.push(0.0);
                            feasiblePoints.push(bPIF);
                            let mut _point = 4000.0 / (j as f64) ;
                            if (robottravel_time -(j as f64)/(self.rules.TICKS_PER_SECOND as f64)).abs() <= 0.02 {
                                _point += 10.0;
                                feasiblePointsTickDiff.push(0.0);
                            } else {
                                feasiblePointsTickDiff.push(((j as f64)/ (self.rules.TICKS_PER_SECOND as f64)) - robottravel_time - 0.05);
                            }
                            if ballPath[j].x.abs() <= self.rules.arena.goal_width/2.0 - 1.0 && kMode!=kickMode::clearDanger && ballPath[j].y >= 10.0 {
                                // _point +=  ballH[j] + ball_speed_after_touch/5.0;
                                if ballPath[j].y >= 0.0 {
                                    _point = _point + 25.0;
                                }
                            }

                            feasiblePointsScore.push(_point);
                            feasiblePointsHeight.push (ballH[j]);
                            //println!("tick needed: {} , time for walk {}, time for jump {}", j, self.travel_time(&(bPIF - (robot_pos - bPIF).normalize()*temp_distBeforJump)) , (temp_distBeforJump/x_speed));
                            feasiblePointsTickNum.push(j as f64);
                            let meCopy = self.me.clone();
                            if self.if_jump_can_touch_point(&meCopy, 15.0, BC.position3(), best_touch_point, &kMode, false) {
                                feasiblePointsTickBJ.push(0.0);//(j as f64) - jump_time * (self.rules.TICKS_PER_SECOND as f64));
                                feasiblePointsJumptSpeed.push(self.rules.ROBOT_MAX_JUMP_SPEED);
                            } else {
                                feasiblePointsTickBJ.push(1000.0);//(j as f64) - jump_time * (self.rules.TICKS_PER_SECOND as f64));
                                feasiblePointsJumptSpeed.push(0.0);
                            }
                            found_best_sol = true;
                        } else {
                            // self.myDrawer.draw(best_touch_point,0.5,(0.0,1.0,1.0));
                            let actual_time = (j as f64)/ (self.rules.TICKS_PER_SECOND as f64);
                            let ballCopy = self.ball_future[j].clone();
                            let kModeCopy = kMode.clone();
                            let res = self.god_simulation(ballCopy, best_touch_point,Vec3::new((*target).x,(*target).y,ballH[j]),actual_time, kModeCopy );

                            if res.0 == true {//} && (res.1.toVec2().normalize().inner_product(&(newTarget.toVec2() - ballCopy.position()).normalize()) > 0.5 || res.1.y >= 5.0){
                                wtfIsThis = true;
                                feasiblePointsMaxSpeed.push(0.0);
                                feasiblePoints.push(bPIF);
                                let mut _point = 4000.0 / (j as f64) ;

                                feasiblePointsTickDiff.push(res.4);
                                if ballPath[j].x.abs() <= self.rules.arena.goal_width/2.0 - 1.0 && kMode!=kickMode::clearDanger && ballPath[j].y >= 10.0 {
                                    _point +=  25.0+ball_speed_after_touch/2.0;


                                }

                                feasiblePointsScore.push(_point);
                                feasiblePointsHeight.push (best_touch_point.h);
                                feasiblePointsTickNum.push(j as f64);
                                feasiblePointsJumptSpeed.push(res.2);
                                feasiblePointsTickBJ.push(res.5);
                                found_best_sol = true;
                            }
                        }


                    }

                }
                // println!("number of feasiblePoints {}",feasiblePoints.len() );


                for i in 0..feasiblePoints.len() {
                    if feasiblePointsScore[i] >= biggestPoint {
                        biggestPoint = feasiblePointsScore[i];
                        tochPoint = feasiblePoints[i];
                        waitForBall = feasiblePointsTickDiff[i];
                        bestHeight = feasiblePointsHeight[i] ;
                        bestTick = feasiblePointsTickNum[i] as usize;
                        bestJumpSpeed = feasiblePointsJumptSpeed[i];
                        best_tick_beforeJump = feasiblePointsTickBJ[i];
                    }
                }
                if (feasiblePoints.len() == 2) {
                    if(kMode == kickMode::clearDanger) {
                        tochPoint = Vec2::new(0.0,self.rules.arena.depth/-2.0 + 1.0);
                    }
                }

                self.myDrawer.draw(Vec3::new(tochPoint.x,tochPoint.y,bestHeight),1.0,(0.0,1.0,1.0));

                if best_tick_beforeJump < 1.0 {
                    jump = bestJumpSpeed;
                } else {
                    jump = 0.0;
                }
                idealPath = (tochPoint - robot_pos).th().deg();

            }
            else {
                jump = 0.0;
                idealPath = (idealPath + shift);
            }
            ////println!("jump {},, result {}, nesbat {}",jump,self.me.velocity3().h, self.me.velocity3().h / jump);

            let mut t_change_dir = 0.0;
            let step_time = 1.0/ ((self.rules.TICKS_PER_SECOND as f64));

            if false &&  self.game.ball.position().y + self.game.ball.velocity().y <= -20.0 && kMode == kickMode::shotForGoal && tochPoint.y <= -20.0{
                Self::gtp(&(Vec2::new(0.0,-10.0)), &self.me, &self.rules, &mut self.action);
            }
            else if waitForBall > step_time{
                let maxSpeedDist = self.rules.ROBOT_MAX_GROUND_SPEED*self.rules.ROBOT_MAX_GROUND_SPEED / self.rules.ROBOT_ACCELERATION ;
                if kMode!= kickMode::clearDanger {
                    tochPoint.y -= maxSpeedDist;
                    Self::gtp(&tochPoint, &self.me, &self.rules, &mut self.action);
                } else {
                    let y_goal = self.rules.arena.depth/-2.0 + 1.0;
                    if(tochPoint.x.abs() > self.rules.arena.goal_width/2.0) {
                        Self::gtp(&(Vec2::new(0.0,y_goal)), &self.me, &self.rules, &mut self.action);
                    } else {
                        Self::gtp(&tochPoint, &self.me, &self.rules, &mut self.action);
                    }
                }
                self.myDrawer.draw(Vec3::new(tochPoint.x,tochPoint.y,bestHeight),1.5,(1.0,1.0,1.0));

            } else {
                if !self.me.touch {
                    jump = 0.0;
                }
                if self.me.position3().dist(self.game.ball.position3()) <= 3.5 && !self.me.touch{
                    jump = 15.0;
                }
                Self::set_robot_vel(idealPath*DEG2RAD ,100.0,jump, &mut self.action);
            }
        }
    }

    fn pm(&mut self, target: &Vec2) {
        for i in 0..self.robot_hist.len() {
            self.myDrawer.draw(self.robot_hist[i].position3() , 1.0 , (0.5,0.5,0.5) );
        }
        let mut newTarget = *target;
        if self.game.ball.position().x > self.rules.arena.width / 2.0 - 2.0 {
            newTarget.x = self.rules.arena.width / 2.0 - 2.0;
        }
        if self.game.ball.position().x < self.rules.arena.width / -2.0  + 2.0 {
            newTarget.x = self.rules.arena.width / -2.0 +2.0;
        }

        // if self.game.ball.position().y + self.game.ball.velocity().y > -20.0 {
            self.kick(&newTarget,kickMode::shotForGoal);
        // } else {
        //     Self::gtp(&(Vec2::new(0.0,-10.0)), &self.me, &self.rules, &mut self.action);
        // }

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
        Self::set_robot_vel(angle, 3.0 * dist , 0.0, action);


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
