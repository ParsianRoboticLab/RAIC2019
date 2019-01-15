use crate::model::*;
use crate::strategy::Strategy;
use std::time::{Duration, Instant};

const TWO_PI : f64 = 2.0 * std::f64::consts::PI;
const EPSILON : f64 = 1.0e-6;
const BALL_PREDICTION_TICKS : usize = 150;
const CAN_DRAW : bool = false;
const GLOBAL_STEP_TIME : f64 = 1.0/ ((60 as f64));

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
    robot_hist_0 : Vec<Robot>,
    robot_hist_1 : Vec<Robot>,
    me : Robot,
    real_me_0 :Robot,
    real_me_1 :Robot,
    is_ball_path_generated : bool,
    rules: Rules,
    game: Game,
    action: Action,
    height_c: usize,
    ball_future: [Ball ; BALL_PREDICTION_TICKS],
    my_drawer : drawer,
    start: Instant,
    tick: i32,
}

impl Default for MyStrategy {
    fn default() -> Self {
        Self {
            me: Robot{..Default::default()},
            is_ball_path_generated : false,
            real_me_0 : Robot{..Default::default()},
            real_me_1 : Robot{..Default::default()},
            rules: Rules{..Default::default()},
            game: Game{..Default::default()},
            action: Action{..Default::default()},
            height_c: 0,
            robot_hist_0 : vec![Robot::default() ; 1],
            robot_hist_1 : vec![Robot::default() ; 1],
            ball_future: [Ball::default(); BALL_PREDICTION_TICKS],
            my_drawer : drawer::default(),
            start: Instant::now(),
            tick : 0
        }
    }
}


impl Strategy for MyStrategy {

    fn act(&mut self, me: &Robot, _rules: &Rules, _game: &Game, _action: &mut Action) {
        // *_action = Action {
        //     target_velocity_x: 30.0,
        //     target_velocity_y: 0.0,
        //     target_velocity_z: 0.0,
        //     jump_speed: 0.0,
        //     use_nitro: false,
        // };
        // return;
        let now = Instant::now();
        self.tick += 1;
        self.me = me.clone();
        self.rules = _rules.clone();
        self.game = _game.clone();
        if self.is_ball_path_generated == false {
            self.update_ball_path();
            self.is_ball_path_generated = true;
        } else {
            self.is_ball_path_generated = false;
        }

        let opp_goal = Vec2::new(0.0, self.rules.arena.depth/2.0 + 2.0);

        if self.me.id == 1 || self.me.id == 3 {
            let mut game_copy = self.game.clone();
            Simulation::tick(self.me.id, &mut game_copy, &self.action, &self.rules);
            for r in &mut game_copy.robots {
                if r.id == self.me.id {
                    self.real_me_0 = r.clone();
                }
            }

        } else {
            let mut game_copy = self.game.clone();
            Simulation::tick(self.me.id, &mut game_copy, &self.action, &self.rules);
            for r in &mut game_copy.robots {
                if r.id == self.me.id {
                    self.real_me_0 = r.clone();
                }
            }
        }

        for r in &_game.robots {
            if r.is_teammate && r.id != me.id {
                if me.position().y < r.position().y {
                    self.gk();
                } else {
                    self.pm(&opp_goal);
                }
                break;
            }
        }
        *_action = self.action;
        println!("LOOP TIME: {:?}", now.elapsed());
        println!("TIME PASS: {:?}", self.start.elapsed());
        println!("TICK LEFT: {:?}", self.rules.max_tick_count - self.tick);
        if self.me.id == 1 || self.me.id == 3 {
            let me_copy = self.me.clone();
            self.robot_hist_0.insert(0,me_copy);
            self.robot_hist_0.truncate(100);
        } else {
            let me_copy = self.me.clone();
            self.robot_hist_1.insert(0,me_copy);
            self.robot_hist_1.truncate(100);
        }


    }


    fn custom_rendering(&mut self) -> String {
        self.my_drawer.createFinalString()
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
enum KickMode {
    ChanceCreation,
    ClearDanger,
    ShotForGoal,
}
#[allow(non_snake_case)]
impl MyStrategy {

    fn update_ball_path(&mut self) {
        self.ball_future = Simulation::get_ball_path(self.me.id, &self.game, &self.rules);
        // for i in 0..self.ball_future.len() {
        //     self.my_drawer.draw(self.ball_future[i].position3(), self.rules.BALL_RADIUS, (0.5,0.5,0.5));
        // }
    }

    fn gk(&mut self) {

        let clear_spot = (self.game.ball.position() - self.me.position()).normalize() * 100.0;
        let y_goal = -self.rules.arena.depth/2.0 - 2.0;
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
            //     self.kick(&Vec2::new(0.0, 100.0), KickMode::ClearDanger);
            let opp_goal = Vec2::new(0.0, self.rules.arena.depth/2.0 + 2.0);

            if self.game.ball.position().y < self.me.position().y {
                self.kick(&opp_goal, KickMode::ClearDanger);
            } else  {
                self.kick(&clear_spot, KickMode::ClearDanger);
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



            Self::gtp(&target, &self.me, &self.rules, &mut self.action);
            self.action.jump_speed = 0.0;
            if ball_pos.dist(self.me.position()) < 3.0 && self.game.ball.height() > 2.5 {
                self.action.jump_speed = self.rules.ROBOT_MAX_JUMP_SPEED;
            }
            let ballHeight = self.game.ball.height();
            let robotCopy = self.me.clone();
            if self.if_jump_can_touch_point(&robotCopy,15.0,Vec3::new(ball_pos.x,ball_pos.y,ballHeight),Vec3::new(ball_pos.x,ball_pos.y,ballHeight),&(KickMode::ClearDanger),false) {
                self.action.jump_speed = self.rules.ROBOT_MAX_JUMP_SPEED;
            }

        }
    }

    fn ballTouchPrediction(&mut self) -> Vec2 {
        let gravity = self.rules.GRAVITY;
        let ball_pos = self.game.ball.position();
        let ballHeight = self.game.ball.height();
        let ball_vel = self.game.ball.velocity();
        let ballhVel = self.game.ball.hVel();
        let timeToTouchTheField = (ballhVel + (ballhVel*ballhVel + 2.0*gravity*(ballHeight - self.game.ball.radius)).sqrt()) / gravity;

        let lenTravel = timeToTouchTheField * ball_vel.len();
        if ball_vel.len() == 0.0 {
            ball_pos
        }
        else {
            ball_pos + ball_vel.normalize() * lenTravel
        }
    }
    fn travel_time(&mut self, target: &Vec2) -> f64 {
        let mut me = self.real_me_0.clone();
        let mut last_me = self.robot_hist_0[0].clone();
        if self.me.id == 2 || self.me.id == 4 {
            last_me = self.robot_hist_1[0].clone();
            me = self.real_me_1.clone();
        }
        let robot_pos = me.position();
        let robot_vel = me.velocity();
        let effectiveSpeed = me.velocity().inner_product(&((*target - me.position()).normalize()));
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
        timeR
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
        timeR
        // 12.5
    }

    fn travel_time_it_alt(&mut self,_mePos : Vec2, _meVel: Vec2, target: &Vec2) -> f64 {
        let mut velocity = _meVel;
        let mut position = _mePos;
        let mut result = 10000.0;
        let perS = 5.0;
        let step_time = (1.0/ ((self.rules.TICKS_PER_SECOND as f64)*perS));
        for i in 0..BALL_PREDICTION_TICKS*5 {
            let idealPath = (*target - position).th().deg()*DEG2RAD;
            if position.dist(*target) <= 0.15{
                return (i as f64) * step_time;
            }
            if velocity.normalize().inner_product(&(*target - position).normalize()) > 0.95 {
                let oldTime = self.travel_time_alt(position, velocity, target);
                return (i as f64) * step_time + oldTime;
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
        return self.travel_time(target);

    }

    fn vel_time_it(&mut self, _meVel: Vec2, targetVel: &Vec2) -> f64 {
        let mut velocity = _meVel;
        let mut result = 10000.0;
        let perS = 2.0;
        let step_time = 2.0/ ((self.rules.TICKS_PER_SECOND as f64)*perS);
        let acc = self.rules.ROBOT_ACCELERATION;
        for i in 0..BALL_PREDICTION_TICKS*2 {

            let target_vel_change = *targetVel - velocity;
            if target_vel_change.len() < (target_vel_change.normalize() * acc * step_time).len(){
                return (i as f64) * step_time;
            }
            if target_vel_change.len() > 0.0 {
                // * _me.touch_normal_y.unwrap().max(0.0);
                velocity = velocity +target_vel_change.normalize() * acc * step_time;
                if velocity.len() > self.rules.ROBOT_MAX_GROUND_SPEED {
                    velocity = velocity.normalize() *self.rules.ROBOT_MAX_GROUND_SPEED;
                }
            }

        }
        let effectiveSpeed = _meVel.inner_product(&targetVel.normalize());

        (targetVel.len() - effectiveSpeed).abs()/self.rules.ROBOT_ACCELERATION

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
            // self.my_drawer.draw(Vec3::new(position.x,position.y,1.0),0.5,(1.0,1.0,1.0));
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
        let step_time = GLOBAL_STEP_TIME;
        position = position + velocity * step_time;
        position.h = position.h - (self.rules.GRAVITY * step_time * step_time / 2.0);
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
                velocity.h = _j_speed*0.9637;

            }
        }

        return (position,velocity);

    }
    fn god_step_gp(&mut self,_position : Vec3 , _velocity : Vec3 , _target : Vec2 , _j_speed : f64 ) -> (Vec3,Vec3) {
        let mut velocity = _velocity;
        let mut position = _position;
        let step_time = GLOBAL_STEP_TIME;
        let target = MyStrategy::gtp_target_validation(_target, &self.rules, &self.me);
        let _rules = self.rules.clone();
        let dist = _position.toVec2().dist(target);
        let mut _ac_speed = (2.0*self.rules.ROBOT_ACCELERATION*dist).sqrt();
        if _ac_speed >= self.rules.ROBOT_MAX_GROUND_SPEED {
            _ac_speed = self.rules.ROBOT_MAX_GROUND_SPEED;
        }
        if dist <= 0.02 {
            _ac_speed = 0.0;
        }
        position = position + velocity * step_time;
        position.h = position.h - (self.rules.GRAVITY * step_time * step_time / 2.0);
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
    fn gtp_action(&self,mee : &Robot, target_main: &Vec2, action: &mut Action) {
        let mut me_copy = mee.clone();
        let mut ball_copy = self.game.ball.clone();
        Simulation::tick(&mut me_copy, &mut ball_copy, action, &self.rules);
        let me = me_copy.clone();
        let target = MyStrategy::gtp_target_validation(*target_main, &self.rules, &me);
        let dist = me.position().dist(target);
        let diff = target - me.position();
        let angle = (diff.y).atan2(diff.x);
        let mut vel = (2.0*(self.rules.ROBOT_ACCELERATION )*dist).sqrt();
        if dist <= 0.05 {
            vel = 0.0;
        }
        if vel >= self.rules.ROBOT_MAX_GROUND_SPEED {
            vel = self.rules.ROBOT_MAX_GROUND_SPEED;
        }
        Self::set_robot_vel(angle, vel, 0.0,action);
    }
    fn god_simulation (&mut self , _ball : Ball,_touch_point : Vec3,_target : Vec3, _time_availabe : f64, _k_mode : KickMode) -> (bool,Vec3,f64,f64,f64,f64){
        let mut me =  self.me.clone();
        let mut ballCopy = self.game.ball.clone();
        let step_time = GLOBAL_STEP_TIME;
        let mut result = (false,Vec3::new(0.0,0.0,0.0),0.0,0.0,0.0,0.0);
        let mut _vir_robot_target = Vec2::new(0.0,0.0);
        let mut jump_tick_time = 0.0;
        let mut best_jump_speed = 0.0;
        let mut best_speed = 0.0;
        let mut waste_time = 1000.0;
        let mut can_jump = 1000.0;
        let mut selected_path = [Vec3::new(0.0,0.0,0.0) ;600];
        let mut virtualAct = Action::default();
        let mut can_touch_ball = false;
        let mut jumpAddedTime = 0.0;
        let mut distNeedForJump =0.0;
        let mut _time_to_reach = 0.0;
        for i in 0..(((_time_availabe)*(self.rules.TICKS_PER_SECOND as f64)) as usize + 20) {
            selected_path[i] = me.position3();
            let _time = (i as f64) / ((self.rules.TICKS_PER_SECOND as f64) );
            let mut _time_to_reach_vel = 0.0;
            ////////////////
            if !(me.position3().h > self.me.radius && !me.touch) {
                jump_tick_time = 10000.0; ;
                best_jump_speed = 0.0;
                best_speed = 0.0;
                jumpAddedTime = 10000.0;
                distNeedForJump = 10000.0;
                // consider jump before everything

                for xSpeedFor in ((me.velocity().len() as usize)..((self.rules.ROBOT_MAX_GROUND_SPEED * 1.0) as usize) + 1).rev() {
                    let j_speed = self.rules.ROBOT_MAX_JUMP_SPEED *0.9637;
                    let mut x_speed = xSpeedFor as f64;
                    x_speed /= 1.0;
                    if x_speed == 0.0 {
                        x_speed = 0.000001;
                    }
                    if ((j_speed*j_speed)/(x_speed*x_speed) - 2.0*((self.rules.GRAVITY*(_touch_point.h - self.me.radius))/(x_speed*x_speed))) < 0.0 {
                        continue;
                    }
                    let temp_distBeforJumpDown = ((j_speed/x_speed) + ((j_speed*j_speed)/(x_speed*x_speed) - 2.0*((self.rules.GRAVITY*(_touch_point.h - self.me.radius))/(x_speed*x_speed))).sqrt()) / (self.rules.GRAVITY/(x_speed*x_speed));
                    let jumptimeDown = (j_speed + (j_speed*j_speed - 2.0*self.rules.GRAVITY*(_touch_point.h - self.me.radius)).sqrt()) / self.rules.GRAVITY;
                    ///// jump with maximum jump with maximum speed
                    let effectiveSpeed = me.velocity().inner_product(&(_touch_point.toVec2() - me.position()).normalize());
                    let distNeededForThisSpeed = (x_speed*x_speed - effectiveSpeed*effectiveSpeed)/(2.0*self.rules.ROBOT_ACCELERATION);

                    let temp_distBeforJumpUP = ((j_speed/x_speed) - ((j_speed*j_speed)/(x_speed*x_speed) - 2.0*((self.rules.GRAVITY*(_touch_point.h - self.me.radius))/(x_speed*x_speed))).sqrt()) / (self.rules.GRAVITY/(x_speed*x_speed));
                    let jumptimeUP = (j_speed - (j_speed*j_speed - 2.0*self.rules.GRAVITY*(_touch_point.h - self.me.radius)).sqrt()) / self.rules.GRAVITY;
                    let mut temp_distBeforJump = 10000.0;
                    let mut jump_time = 10000000.0;
                    temp_distBeforJump = jumptimeUP * x_speed;
                    jump_time= jumptimeUP;

                    // if jumptimeUP < 0.0 {
                    //     temp_distBeforJump = temp_distBeforJumpDown;
                    //     jump_time= jumptimeDown;
                    // }
                    if jump_time < 0.0 {
                        temp_distBeforJump = 10000.0;
                        jump_time = 10000.0;
                    }
                    _time_to_reach_vel = self.vel_time_it(me.velocity(), &((_touch_point.toVec2() - me.position()).normalize()*x_speed));

                    if _time_availabe - _time_to_reach_vel - jump_time - _time >= 0.0 && temp_distBeforJump <= me.position().dist(_touch_point.toVec2()){
                        jump_tick_time = _time_availabe - jump_time;
                        best_jump_speed = j_speed / 0.9637;
                        best_speed = x_speed;
                        jumpAddedTime = jump_time;
                        distNeedForJump = temp_distBeforJump;
                        break;
                    }
                }
            }
            if jump_tick_time < 2.0*step_time{//} && ((self.me.velocity().len() - best_speed).abs() < step_time*self.rules.ROBOT_ACCELERATION || self.me.velocity().len() < 0.1){
                can_jump = 0.0;
            }
            _time_to_reach = self.travel_time_it_alt(me.position(), me.velocity(), &(_touch_point.toVec2() + (me.position() - _touch_point.toVec2()).normalize()*distNeedForJump));
            let jumpPoint = _touch_point.toVec2() + (me.position() - _touch_point.toVec2()).normalize()*distNeedForJump;
            // self.my_drawer.drawText(format!("time to reach :: {}", _time_to_reach_vel));
            let extera_time =  _time_availabe - _time_to_reach - jumpAddedTime - _time;

            if _time_availabe - _time_to_reach - jumpAddedTime <= step_time  &&  i == 0 {
                waste_time = 0.0;
            }
            self.my_drawer.drawText(format!(" jump tick time - time {} best_speed {} vel {} extera_time {} gPA? {}",jump_tick_time , best_speed , me.velocity().len(),extera_time,extera_time > step_time || distNeedForJump == 10000.0));

            if  extera_time > step_time || distNeedForJump == 10000.0{
                let maxSpeedDist = self.rules.ROBOT_MAX_GROUND_SPEED*self.rules.ROBOT_MAX_GROUND_SPEED / self.rules.ROBOT_ACCELERATION ;
                let mut altPoint = _touch_point.toVec2() - (Vec2::new(0.0,10.5));
                if _k_mode == KickMode::ClearDanger{
                    let y_goal = self.rules.arena.depth/-2.0 + 1.0;
                    altPoint = Vec2::new(0.0,y_goal);
                }
                self.my_drawer.draw(me.position3(),0.5,(1.0,0.0,0.0));
                self.gtp_action(&me, &altPoint, &mut virtualAct);
            } else {
                self.my_drawer.draw(me.position3(),0.5,(0.0,0.0,1.0));

                self.my_drawer.draw(Vec3::new((_touch_point.toVec2() + (me.position() - _touch_point.toVec2()).normalize()*distNeedForJump).x,(_touch_point.toVec2() + (me.position() - _touch_point.toVec2()).normalize()*distNeedForJump).y,2.0),1.0,(1.0,1.0,1.0));
                let mut _j_for_kick = 0.0;

                if _time >= jump_tick_time - step_time{//} && ((me.velocity().len() - best_speed).abs() < step_time*self.rules.ROBOT_ACCELERATION || me.velocity().len() < 0.1)  {
                    _j_for_kick = best_jump_speed;
                }
                // println!("best JS {}" , _j_for_kick);
                let idealPath = (_touch_point.toVec2() - me.position()).th().deg()*DEG2RAD;
                Self::set_robot_vel(idealPath,self.rules.ROBOT_MAX_GROUND_SPEED, _j_for_kick,&mut virtualAct);
            }
            let mut game_copy = self.game.clone();
            Simulation::tick(me.id, &mut game_copy, &virtualAct, &self.rules);

            if me.position3().dist(_ball.position3()) < self.me.radius + self.game.ball.radius {
                can_touch_ball = true;
            }
            if (me.position3().dist(_touch_point) < self.rules.ROBOT_MAX_GROUND_SPEED*step_time*20000.0 || _k_mode == KickMode::ClearDanger) && can_touch_ball == true {
                // for j in 0..(((_time_availabe)*(self.rules.TICKS_PER_SECOND as f64)) as usize + 10) {
                //     self.my_drawer.draw(selected_path[j],0.5,(1.0,0.0,0.0));
                // }
                self.my_drawer.drawText(format!("realVel {}, simVel {}, best Speed {}, js {} , can_jump {} , best_JS {}",self.me.velocity().len(),me.velocity().len(),best_speed,best_jump_speed,can_jump,best_jump_speed));

                result.5 = can_jump;
                result.4 = waste_time;
                result.3 = extera_time;
                result.2 = self.rules.ROBOT_MAX_JUMP_SPEED;
                result.0 = true;
                break;
            }

        }
        result
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

    //TODO: calc true jump speed
    fn if_jump_can_touch_point(&mut self,robot : &Robot, jump_speed : f64, _target : Vec3, _touchPoint : Vec3, kMode:& KickMode , _aggresive : bool) -> bool {
        let pos = robot.position3();
        let vel = robot.velocity3();
        let mut virtualPos = pos;
        let mut virtualVel = vel;
        if virtualVel.h == 0.0 {
            virtualVel.h = jump_speed*0.9637;
        }
        let maxTime = 2.0*virtualVel.h/self.rules.GRAVITY;
        let delta_time = 0.01;
        let target = _target;

        for _i in 0..((100.0*maxTime) as i64) -1 {

            virtualPos += virtualVel * delta_time;
            virtualPos.h -= self.rules.GRAVITY*delta_time*delta_time/2.0;
            virtualVel.h -= self.rules.GRAVITY * delta_time;
            if  virtualPos.dist(target) <= self.me.radius +self.game.ball.radius - 0.2{
                return true;
            }
        }
        return false;
    }
    fn best_place_on_ball_for_kick(&mut self, finalVel : Vec3, _ball: &Ball, _radius_change_speed: f64,_rules: &Rules, _k_mode : KickMode) -> (Vec3,f64) {
        let mut _result = Vec3::new(5000.0,5000.0,5000.0);
        let _finalVelNormm = finalVel.normalize();
        let mut _x = 0.0;
        let mut _y = 0.0;
        let mut _z = 0.0;
        let mut _theta = 0.0;
        let mut _phi:f64 = 0.0;
        let mut robot_max_vel = 0.0;
        let mut _impulseVec = Vec3::new(0.0,0.0,0.0);
        let _radius = _ball.radius + self.me.radius;
        let mut _outPut = Vec3::new(0.0,0.0,0.0);
        let mut bestAnswer = Vec3::new(0.0,0.0,0.0);
        let mut answerDistTheta = 1000000.0;
        let mut ball_vel_after_touch = 0.0;
        let kick_path_theta = finalVel.toVec2().th().deg() as i64;
        bestAnswer.h = _ball.position3().h - finalVel.h;
        if _k_mode == KickMode::ClearDanger {
            bestAnswer.h = _ball.position3().h - self.me.radius - 0.6;
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
            //should be realFinalVel
            let tan_speed = (virtualBot.position() - self.me.position() ).normalize().inner_product(&(self.me.velocity()));
            if tan_speed >= self.rules.ROBOT_MAX_GROUND_SPEED {
                robot_max_vel = self.rules.ROBOT_MAX_GROUND_SPEED;
            } else {
                robot_max_vel = (2.0*self.rules.ROBOT_ACCELERATION*(*virtualBot).position().dist(self.me.position()) + tan_speed*tan_speed).sqrt();
                if robot_max_vel >= self.rules.ROBOT_MAX_GROUND_SPEED{
                    robot_max_vel = self.rules.ROBOT_MAX_GROUND_SPEED;
                }
            }
            virtualBot.set_velocity(&((virPos - self.me.position3()).normalize()*robot_max_vel));
            _outPut = Simulation::simple_robot_ball_colide_step(virtualBot,_ball,_radius_change_speed,_rules);
            let _dist = (_outPut.toVec2().normalize() - _finalVelNormm.toVec2()).len();
            if _dist < answerDistTheta {
                answerDistTheta = _dist;
                bestAnswer = Vec3::new(_x,_y,_z);
                ball_vel_after_touch = _outPut.toVec2().inner_product(&(finalVel.toVec2().normalize()));
            }
        }
        //should add jump speed for phi
        return (bestAnswer,ball_vel_after_touch);
    }

    fn kick(&mut self, target: &Vec2, kMode : KickMode)  {
        let ball_pos = self.game.ball.position();
        let robot_pos = self.me.position();
        let robot_vel = self.me.velocity();
        let finalDir = (*target - ball_pos).th();
        let mut idealPath = (ball_pos - robot_pos).th().deg();
        let movementDir = ((ball_pos - robot_pos).th() - finalDir).normalize().deg();
        let ball_vel = self.game.ball.velocity();
        let mut waitForBall = 0.0;
        let mut me = self.real_me_0.clone();
        let mut last_me = self.robot_hist_0[0].clone();
        if self.me.id == 2 || self.me.id == 4 {
            last_me = self.robot_hist_1[0].clone();
            me = self.real_me_1.clone();
        }
        //println!("movementDir {}", movementDir );

        let mut shift = 0.0;

        if movementDir.abs() < 5.0 {
            shift = 0.0;
        } else if movementDir > 0.0 {
            shift = 50.0 + 30.0/ball_pos.dist(robot_pos);
        } else {
            shift = -50.0 - 30.0/ball_pos.dist(robot_pos);
        }
        let mut jump = 0.0;
        let robotCurrentPath = self.me.velocity().th().deg();
        let mut new_ball = &mut self.game.ball.clone();
        let mut best_ball = &mut self.game.ball.clone();

        ////////
        let mut touch_point = ball_pos + (ball_pos - *target).normalize()*(self.game.ball.radius + self.me.radius - 0.5);
        //kickoff
        let mut biggestPoint = 0.0;
        let mut bestHeight = 0.0;
        let mut bestTick = 0;
        let mut bestJumpSpeed = 0.0;
        let mut best_tick_beforeJump = 0.0;
        if ball_vel.len() <= std::f64::EPSILON {
            idealPath = (touch_point - robot_pos).th().deg();
            if robot_vel.len() > 29.5  {
                jump = self.game.ball.height() *4.0;
            }
            if self.me.height() > 1.2 {
                if robot_pos.dist(ball_pos) < 3.2  {
                    jump = 15.0;
                }
                else {
                    jump = 0.0;
                }
            }
            Self::set_robot_vel(idealPath*DEG2RAD , 100.0 ,jump, &mut self.action);
        } else {
            ////// New prediction
            let mut extera_time = 0.0;
            if self.me.position3().h <= self.me.radius || self.me.touch{// movementDir.abs() < 110.0 || kMode == KickMode::ShotForGoal{
                let mut ballPath = [Vec3::new(0.0,0.0,0.0) ; 600];
                let mut ballH = [0.0 ; 600];
                let mut feasiblePoints = vec! [Vec2::new(0.0,0.0) ; 2];
                let mut feasiblePointsScore = vec![0.0 ; 2];
                let mut feasiblePointsHeight = vec![0.0 ; 2];
                let mut feasiblePointsTickDiff = vec![0.0 ; 2];
                let mut feasiblePointsTickNum = vec![0.0;2];
                let mut feasiblePointsMaxSpeed = vec![0.0;2];
                let mut feasiblePointsJumptSpeed = vec![0.0;2];
                let mut feasiblePointsTickBJ = vec![0.0 ;2];
                let mut feasiblePointsExteraTime = vec![0.0; 2];

                for j in 0..self.ball_future.len() {
                    ballPath[j] = self.ball_future[j].position3();
                    ballH[j] = self.ball_future[j].height();
                    let BC = self.ball_future[j].clone();
                    let actual_time = (j as f64 )/ (self.rules.TICKS_PER_SECOND as f64);
                    // TODO: using a good method for calculate target bestHeight
                    let mut hhh = 70.0;
                    if BC.position().y > 10.0 {
                        hhh = 7.0;
                    }
                    let mut newTarget = Vec3::new((*target).x,(*target).y + 2.0,hhh);


                    if BC.position3().x.abs() < self.rules.arena.width / 2.0 - 4.0 {
                        newTarget.x = BC.position3().x;
                    }
                    let reflect_target = self.calc_point_for_reflect_kick(BC, newTarget);
                    // find the best soloution for kick direct or using reflection
                    if ((ballPath[j].toVec2() - robot_pos).normalize().inner_product(&(newTarget.toVec2() - ballPath[j].toVec2()).normalize()) <
                    (ballPath[j].toVec2() - robot_pos).normalize().inner_product(&(reflect_target.toVec2() - ballPath[j].toVec2()).normalize()))
                    && kMode==KickMode::ShotForGoal && ballPath[j].y < self.rules.arena.depth / 2.0 - 30.0 && kMode != KickMode::ClearDanger{
                        newTarget = reflect_target;
                    }

                    let bestVec = (newTarget - ballPath[j]).normalize();
                    let rulesCopy = self.rules.clone();
                    let fd =  (newTarget.toVec2() - BC.position()).th();
                    let md =  ((BC.position() - robot_pos).th() - finalDir).normalize().deg();
                    let mut best_touch_point = Vec3::new(0.0,0.0,0.0);
                    if  true || md < 90.0 && best_touch_point.y > 20.0 {
                        best_touch_point =  self.best_place_on_ball_for_kick(bestVec,&(BC),rulesCopy.ROBOT_MAX_JUMP_SPEED,&(rulesCopy),kMode).0;

                    } else {
                        best_touch_point = BC.position3() + (BC.position3() - newTarget).normalize()*(self.me.radius + self.game.ball.radius );
                    }
                    let mut best_pos_in_the_future = best_touch_point.toVec2();
                    let me_copy = self.me.clone();
                    let robottravel_time = self.travel_time_it_alt(me_copy.position(),me_copy.velocity(),&(best_touch_point.toVec2()));
                    //// maximum ball height for feasible points
                    let maximum_ball_height = 4.4;
                    //// find points that have potential of being feasible
                    if (robottravel_time -actual_time) <= GLOBAL_STEP_TIME  && best_touch_point.h < maximum_ball_height {

                        self.my_drawer.draw(best_touch_point,0.5,(1.0,1.0,1.0));
                        self.my_drawer.draw(BC.position3(),2.0,(0.0,1.0,0.0));
                        let ballCopy = self.ball_future[j].clone();
                        let kModeCopy = kMode.clone();
                        let res = self.god_simulation(ballCopy, best_touch_point,newTarget,actual_time, kModeCopy );
                        //// check if that point is really feasible or not
                        if res.0 == true {///} && (res.1.toVec2().normalize().inner_product(&(newTarget.toVec2() - ballCopy.position()).normalize()) > 0.5 || res.1.y >= 5.0){
                            feasiblePointsMaxSpeed.push(0.0);
                            feasiblePoints.push(best_pos_in_the_future);
                            let mut _point = 4000.0 / (j as f64) ;
                            feasiblePointsTickDiff.push(res.4);
                            feasiblePointsScore.push(_point);
                            feasiblePointsHeight.push (best_touch_point.h);
                            feasiblePointsTickNum.push(j as f64);
                            feasiblePointsJumptSpeed.push(res.2);
                            feasiblePointsTickBJ.push(res.5);
                            feasiblePointsExteraTime.push(res.3);
                            break;
                        }
                        break;

                    }
                }
                //// find best feasible point
                for i in 0..feasiblePoints.len() {
                    if feasiblePointsScore[i] >= biggestPoint {
                        biggestPoint = feasiblePointsScore[i];
                        touch_point = feasiblePoints[i];
                        waitForBall = feasiblePointsTickDiff[i];
                        bestHeight = feasiblePointsHeight[i] ;
                        bestTick = feasiblePointsTickNum[i] as usize;
                        bestJumpSpeed = feasiblePointsJumptSpeed[i];
                        best_tick_beforeJump = feasiblePointsTickBJ[i];
                        extera_time = feasiblePointsExteraTime[i];
                    }
                }
                if feasiblePoints.len() == 2 && kMode == KickMode::ClearDanger{
                    touch_point = Vec2::new(0.0,self.rules.arena.depth/-2.0 + 1.0);
                }
                self.my_drawer.draw(Vec3::new(touch_point.x,touch_point.y,bestHeight),1.0,(0.0,1.0,1.0));
                //// find out jump or not
                if best_tick_beforeJump < 1.0 {
                    jump = bestJumpSpeed;
                } else {
                    jump = 0.0;
                }
                idealPath = (touch_point - robot_pos).th().deg();
            }
            else {
                // jTurn
                jump = 0.0;
                idealPath = idealPath + shift;
            }
            ////// if robot reach to the best point faster than ball
            if waitForBall > GLOBAL_STEP_TIME{
                let maxSpeedDist = self.rules.ROBOT_MAX_GROUND_SPEED*self.rules.ROBOT_MAX_GROUND_SPEED / self.rules.ROBOT_ACCELERATION ;
                touch_point = touch_point - (Vec2::new(0.0,10.5));
                if (kMode == KickMode::ClearDanger) {
                    let y_goal = self.rules.arena.depth/-2.0 + 1.0;
                    touch_point = Vec2::new(0.0,y_goal);
                }

                self.gtpSelf(&touch_point);

                //
                // if kMode!= KickMode::ClearDanger {
                //     Self::gtp(&touch_point, &self.me, &self.rules, &mut self.action);
                // } else {
                //     if touch_point.x.abs() > self.rules.arena.goal_width/2.0 - self.game.ball.radius {
                //         Self::gtp(&(Vec2::new(0.0,y_goal)), &self.me, &self.rules, &mut self.action);
                //     } else {
                //         Self::gtp(&touch_point, &self.me, &self.rules, &mut self.action);
                //     }
                // }
                self.my_drawer.draw(Vec3::new(touch_point.x,touch_point.y,bestHeight),1.5,(1.0,1.0,1.0));


            } else {
                if !me.touch {
                    jump = 0.0;
                }
                if me.position3().dist(self.game.ball.position3()) <= self.me.radius + self.game.ball.radius + 0.3 && self.me.position3().h > self.me.radius{
                    jump = 15.0;
                }
                Self::set_robot_vel(idealPath*DEG2RAD ,self.rules.ROBOT_MAX_GROUND_SPEED,jump, &mut self.action);
            }
        }
    }

    fn pm(&mut self, target: &Vec2) {

        let mut newTarget = *target;
        if self.game.ball.position().x > self.rules.arena.width / 2.0 - 2.0 {
            newTarget.x = self.rules.arena.width / 2.0 - 2.0;
        }
        if self.game.ball.position().x < self.rules.arena.width / -2.0  + 2.0 {
            newTarget.x = self.rules.arena.width / -2.0 +2.0;
        }
        self.kick(&(newTarget),KickMode::ShotForGoal);
    }
    fn gtp_target_validation (_target: Vec2,_rules: &Rules,me: &Robot) -> Vec2 {
        let mut target = _target;
        if (target).y > _rules.arena.depth / 2.0 - _rules.arena.bottom_radius - me.radius{
            (target).y = _rules.arena.depth / 2.0 - _rules.arena.bottom_radius - me.radius;
        }
        if target.x > _rules.arena.width / 2.0 - _rules.arena.bottom_radius - me.radius {
            target.x = _rules.arena.width / 2.0 - _rules.arena.bottom_radius- me.radius;
        }
        if target.x < _rules.arena.width / -2.0 +_rules.arena.bottom_radius + me.radius{
            target.x = _rules.arena.width / -2.0 + _rules.arena.bottom_radius + me.radius;
        }

        if target.y < _rules.arena.depth / -2.0 + _rules.arena.bottom_radius {
            if target.x > _rules.arena.goal_width/2.0 - _rules.arena.bottom_radius - me.radius {
                target.x = _rules.arena.goal_width/2.0 - _rules.arena.bottom_radius- me.radius;
            }
            if target.x < -_rules.arena.goal_width/2.0 + _rules.arena.bottom_radius + me.radius{
                target.x = -_rules.arena.goal_width/2.0 + _rules.arena.bottom_radius+ me.radius;
            }
        }
        if target.x.abs() < _rules.arena.goal_width/2.0 - _rules.arena.bottom_radius {
            if target.y < _rules.arena.depth / -2.0 - _rules.arena.goal_depth + _rules.arena.bottom_radius +  me.radius{
                target.y = _rules.arena.depth / -2.0 - _rules.arena.goal_depth + _rules.arena.bottom_radius + me.radius;
            }
        } else {
            if target.y < _rules.arena.depth / -2.0 + _rules.arena.bottom_radius - me.radius{
                target.y = _rules.arena.depth / -2.0 + _rules.arena.bottom_radius - me.radius ;
            }
        }
        target
    }
    fn gtp(target_main: &Vec2, me: &Robot, _rules: &Rules, action: &mut Action) {

        let target = MyStrategy::gtp_target_validation(*target_main, _rules, me);
        let dist = me.position().dist(target);
        let diff = target - me.position();
        let angle = (diff.y).atan2(diff.x);
        let mut vel = (2.0*(_rules.ROBOT_ACCELERATION )*dist).sqrt();
        if dist <= 1.0 {
            vel = dist;
        }
        if vel >= _rules.ROBOT_MAX_GROUND_SPEED {
            vel = _rules.ROBOT_MAX_GROUND_SPEED;
        }
        // self.my_drawer.drawText(format!("real vel {} , applied Vel",me.velocity().len(),vel));
        Self::set_robot_vel(angle, vel, 0.0, action);
    }

    fn gtpSelf (&mut self, target_main: &Vec2) {
        let mut me = self.real_me_0.clone();
        let mut last_me = self.robot_hist_0[0].clone();
        if self.me.id == 2 || self.me.id == 4 {
            last_me = self.robot_hist_1[0].clone();
            me = self.real_me_1.clone();
        }
        let target = MyStrategy::gtp_target_validation(*target_main, &self.rules, &me);
        let dist = me.position().dist(target);
        let diff = target - me.position();
        let angle = (diff.y).atan2(diff.x);
        let mut vel = (2.0*(self.rules.ROBOT_ACCELERATION )*dist).sqrt();
        if dist <= 0.02 {
            vel = 0.0;
        }
        if vel >= self.rules.ROBOT_MAX_GROUND_SPEED {
            vel = self.rules.ROBOT_MAX_GROUND_SPEED;
        }

        self.my_drawer.drawText(format!("real vel {} , applied Vel {}, acc {}, hist Vel {}",me.velocity().len(),vel,(me.velocity() - last_me.velocity()).len()/GLOBAL_STEP_TIME,last_me.velocity().len()));
        Self::set_robot_vel(angle, vel, 0.0, &mut self.action);
    }
    fn set_robot_vel(angle : f64, vel: f64, jump : f64, action: &mut Action) {
        println!("sag vel : {}",vel);
        *action = Action {
            target_velocity_x: vel*angle.cos(),
            target_velocity_y: 0.0,
            target_velocity_z: vel*angle.sin(),
            jump_speed: jump,
            use_nitro: false,
        }
    }
}
