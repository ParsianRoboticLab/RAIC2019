use crate::model::*;
use crate::strategy::Strategy;

const TWO_PI : f64 = 2.0 * std::f64::consts::PI;
const EPSILON : f64 = 1.0e-6;

include!("pid.rs");
include!("vec2.rs");
include!("Vec3.rs");
include!("def.rs");
include!("entity.rs");
include!("coach.rs");
include!("angdeg.rs");
include!("seg2.rs");
include!("line2.rs");
include!("simulation.rs");
include!("vec3.rs");
include!("entity3.rs");
include!("circle2.rs");

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

    fn dan_to_plane(point: Vec3, point_on_plane: Vec3, plane_normal: Vec3) -> (f64,Vec3) {
        return (((point - point_on_plane).inner_product(&plane_normal)) , plane_normal);
    }
    fn dan_to_sphere_inner(point: Vec3, sphere_center: Vec3, sphere_radius: f64) -> (f64,Vec3) {
    return (sphere_radius - (point - sphere_center).len(),(sphere_center - point).normalize());
    }
    fn dan_to_sphere_outer(point: Vec3, sphere_center: Vec3, sphere_radius: f64) -> (f64,Vec3) {
    return ((point - sphere_center).len() - sphere_radius,(point - sphere_center).normalize()) ;
    
    }
    fn min(a: (f64,Vec3), b :(f64,Vec3)) -> (f64,Vec3) {
        if a.0 > b.0 {
            return a;
        } else {
            return b;
        }
    }
    fn clamp(_input : f64 , _min : f64 , _max : f64) -> f64{
        if _input > _max {
            return _max;
        } 
        else if _input < _min {
            return _min;
        } else {
            return _input;
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
//// sth smb do
    
    fn dan_to_arena_quarter(&mut self, point: Vec3) -> (f64,Vec3) {
    // Ground
    let mut dan = dan_to_plane(point, Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    // Ceiling
    if dan.0 > dan_to_plane(point, Vec3::new(0.0, self.rules.arena.height, 0.0), Vec3::new(0.0, -1.0, 0.0)).0 {
        dan = dan_to_plane(point, Vec3::new(0.0, self.rules.arena.height, 0.0), Vec3::new(0.0, -1.0, 0.0));
    }
    // Side x
    if dan.0 > dan_to_plane(point, Vec3::new(self.rules.arena.width / 2.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0)).0 {
        dan = dan_to_plane(point, Vec3::new(self.rules.arena.width / 2.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0));
    }
    // Side z (goal)
    if dan.0 >= dan_to_plane(point,Vec3::new(0.0, 0.0, (self.rules.arena.depth / 2.0) + self.rules.arena.goal_depth),Vec3::new(0.0, 0.0, -1.0)).0 {
    dan = dan_to_plane(point,Vec3::new(0.0, 0.0, (self.rules.arena.depth / 2.0) + self.rules.arena.goal_depth),Vec3::new(0.0, 0.0, -1.0));
    }
    // Side z
    let mut v = Vec2::new(point.x, point.y) - Vec2::new((self.rules.arena.goal_width / 2.0) - self.rules.arena.goal_top_radius,self.rules.arena.goal_height - self.rules.arena.goal_top_radius);
    if point.x >= (self.rules.arena.goal_width / 2.0) + self.rules.arena.goal_side_radius || point.y >= self.rules.arena.goal_height + self.rules.arena.goal_side_radius|| (v.x > 0.0 && v.y > 0.0 && v.len() >= self.rules.arena.goal_top_radius + self.rules.arena.goal_side_radius) {        
        dan = min(dan , dan_to_plane(point, Vec3::new(0.0, 0.0, self.rules.arena.depth / 2.0), Vec3::new(0.0, 0.0, -1.0)));
    }
    
    //Side x & ceiling (goal)
    if point.h >= (self.rules.arena.depth / 2.0) + self.rules.arena.goal_side_radius {
    // x
        dan = min(dan, dan_to_plane(point,Vec3::new(self.rules.arena.goal_width / 2.0, 0.0, 0.0),Vec3::new(-1.0, 0.0, 0.0)));
        // y
        dan = min(dan, dan_to_plane(point, Vec3::new(0.0, self.rules.arena.goal_height, 0.0), Vec3::new(0.0, -1.0, 0.0)));
    }
    // Goal back corners

    if point.h > (self.rules.arena.depth / 2.0) + self.rules.arena.goal_depth - self.rules.arena.bottom_radius {
    dan = min(dan, dan_to_sphere_inner(point,Vec3::new(clamp(point.x,self.rules.arena.bottom_radius - (self.rules.arena.goal_width / 2.0),(self.rules.arena.goal_width / 2.0) - self.rules.arena.bottom_radius),clamp(point.y,self.rules.arena.bottom_radius,self.rules.arena.goal_height - self.rules.arena.goal_top_radius,),(self.rules.arena.depth / 2.0) + self.rules.arena.goal_depth - self.rules.arena.bottom_radius),self.rules.arena.bottom_radius));
    }
    // // Corner
    if point.x > (self.rules.arena.width / 2.0) - self.rules.arena.corner_radius && point.h > (self.rules.arena.depth / 2.0) - self.rules.arena.corner_radius{
        dan = min(dan, dan_to_sphere_inner(point,Vec3::new((self.rules.arena.width / 2.0) - self.rules.arena.corner_radius,point.y,(self.rules.arena.depth / 2.0) - self.rules.arena.corner_radius),self.rules.arena.corner_radius));
    }
    // Goal outer corner
    if point.h < (self.rules.arena.depth / 2.0) + self.rules.arena.goal_side_radius {
        // Side x
        if point.x < (self.rules.arena.goal_width / 2.0) + self.rules.arena.goal_side_radius {
            dan = min(dan, dan_to_sphere_outer(point,Vec3::new((self.rules.arena.goal_width / 2.0) + self.rules.arena.goal_side_radius,point.y,(self.rules.arena.depth / 2.0) + self.rules.arena.goal_side_radius),self.rules.arena.goal_side_radius));
        }
        // Ceiling
        if point.y < self.rules.arena.goal_height + self.rules.arena.goal_side_radius {
            dan = min(dan, dan_to_sphere_outer(
            point,
            Vec3::new(
            point.x,
            self.rules.arena.goal_height + self.rules.arena.goal_side_radius,
            (self.rules.arena.depth / 2.0) + self.rules.arena.goal_side_radius
            ),
            self.rules.arena.goal_side_radius));
        }
        // Top corner
        let o = Vec2::new(
        (self.rules.arena.goal_width / 2.0) - self.rules.arena.goal_top_radius,
        self.rules.arena.goal_height - self.rules.arena.goal_top_radius
        );
        let v = Vec2::new(point.x, point.y) - o;
        if v.x > 0.0 && v.y > 0.0 {
            let o = o + v.normalize() * (self.rules.arena.goal_top_radius + self.rules.arena.goal_side_radius);
            dan = min(dan, dan_to_sphere_outer(
            point,
            Vec3::new(o.x, o.y, (self.rules.arena.depth / 2.0) + self.rules.arena.goal_side_radius),
            self.rules.arena.goal_side_radius));
        }
    }
    // Goal inside top corners
    if point.h > (self.rules.arena.depth / 2.0) + self.rules.arena.goal_side_radius
    && point.y > self.rules.arena.goal_height - self.rules.arena.goal_top_radius {
    // Side x
        if point.x > (self.rules.arena.goal_width / 2.0) - self.rules.arena.goal_top_radius {
            dan = min(dan, dan_to_sphere_inner(
            point,
            Vec3::new(
            (self.rules.arena.goal_width / 2.0) - self.rules.arena.goal_top_radius,
            self.rules.arena.goal_height - self.rules.arena.goal_top_radius,
            point.h
            ),
            self.rules.arena.goal_top_radius));
        }
        // Side z
        if point.h > (self.rules.arena.depth / 2.0) + self.rules.arena.goal_depth - self.rules.arena.goal_top_radius{
        dan = min(dan, dan_to_sphere_inner(point,Vec3::new(point.x,self.rules.arena.goal_height - self.rules.arena.goal_top_radius,
        (self.rules.arena.depth / 2.0) + self.rules.arena.goal_depth - self.rules.arena.goal_top_radius
        ),
        self.rules.arena.goal_top_radius));
        }
    }
    if point.y < self.rules.arena.bottom_radius {
        // Side x
        if point.x > (self.rules.arena.width / 2.0) - self.rules.arena.bottom_radius {
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3::new(
                        (self.rules.arena.width / 2.0) - self.rules.arena.bottom_radius,
                        self.rules.arena.bottom_radius,
                        point.h
                    ),
                    self.rules.arena.bottom_radius));
        }
        // Side z
        if point.h > (self.rules.arena.depth / 2.0) - self.rules.arena.bottom_radius
                && point.x >= (self.rules.arena.goal_width / 2.0) + self.rules.arena.goal_side_radius {
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3::new(
                        point.x,
                        self.rules.arena.bottom_radius,
                        (self.rules.arena.depth / 2.0) - self.rules.arena.bottom_radius
                    ),
                    self.rules.arena.bottom_radius));
        }
        // Side z (goal)
        if point.h > (self.rules.arena.depth / 2.0) + self.rules.arena.goal_depth - self.rules.arena.bottom_radius{
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3::new(
                        point.x,
                        self.rules.arena.bottom_radius,
                        (self.rules.arena.depth / 2.0) + self.rules.arena.goal_depth - self.rules.arena.bottom_radius),self.rules.arena.bottom_radius));
        }
        // Goal outer corner
        let o = Vec2::new(
            (self.rules.arena.goal_width / 2.0) + self.rules.arena.goal_side_radius,
            (self.rules.arena.depth / 2.0) + self.rules.arena.goal_side_radius
        );
        let v = Vec2::new(point.x, point.h) - o;
        if v.x < 0.0 && v.y < 0.0 
                && v.len() < self.rules.arena.goal_side_radius + self.rules.arena.bottom_radius{
            let o = o + v.normalize() * (self.rules.arena.goal_side_radius + self.rules.arena.bottom_radius);
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3::new(o.x, self.rules.arena.bottom_radius, o.y),
                    self.rules.arena.bottom_radius));
        }
        // Side x (goal)
        if point.h >= (self.rules.arena.depth / 2.0) + self.rules.arena.goal_side_radius
                && point.x > (self.rules.arena.goal_width / 2.0) - self.rules.arena.bottom_radius {
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3::new(
                        (self.rules.arena.goal_width / 2.0) - self.rules.arena.bottom_radius,
                        self.rules.arena.bottom_radius,
                        point.h
                    ),
                    self.rules.arena.bottom_radius));
        }
        // Corner
        if point.x > (self.rules.arena.width / 2.0) - self.rules.arena.corner_radius
                && point.h > (self.rules.arena.depth / 2.0) - self.rules.arena.corner_radius {
            let corner_o = Vec2::new(
                (self.rules.arena.width / 2.0) - self.rules.arena.corner_radius,
                (self.rules.arena.depth / 2.0) - self.rules.arena.corner_radius
            );
            let n = Vec2::new(point.x, point.h) - corner_o;
            let dist = n.len();
            if dist > self.rules.arena.corner_radius - self.rules.arena.bottom_radius {
                let n = n * (1.0/dist);
                let o2 = corner_o + n * (self.rules.arena.corner_radius - self.rules.arena.bottom_radius);
                dan = min(dan, dan_to_sphere_inner(
                        point,
                        Vec3::new(o2.x, self.rules.arena.bottom_radius, o2.y),
                        self.rules.arena.bottom_radius));
            }
        }
    }
    // Ceiling corners
    if point.y > self.rules.arena.height - self.rules.arena.top_radius {
        // Side x
        if point.x > (self.rules.arena.width / 2.0) - self.rules.arena.top_radius{
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3::new(
                        (self.rules.arena.width / 2.0) - self.rules.arena.top_radius,
                        self.rules.arena.height - self.rules.arena.top_radius,
                        point.h,
                    ),
                    self.rules.arena.top_radius));
        }
        // Side z
        if point.h > (self.rules.arena.depth / 2.0) - self.rules.arena.top_radius{
            dan = min(dan, dan_to_sphere_inner(
                    point,
                    Vec3::new(
                        point.x,
                        self.rules.arena.height - self.rules.arena.top_radius,
                        (self.rules.arena.depth / 2.0) - self.rules.arena.top_radius,
                    ),
                    self.rules.arena.top_radius));
        }

        // Corner
        if point.x > (self.rules.arena.width / 2.0) - self.rules.arena.corner_radius && point.h > (self.rules.arena.depth / 2.0) - self.rules.arena.corner_radius {
            let corner_o = Vec2::new(
                (self.rules.arena.width / 2.0) - self.rules.arena.corner_radius,
                (self.rules.arena.depth / 2.0) - self.rules.arena.corner_radius
            );
            let dv = Vec2::new(point.x, point.h) - corner_o;
            if dv.len() > self.rules.arena.corner_radius - self.rules.arena.top_radius {
                let n = dv.normalize();
                let o2 = corner_o + n * (self.rules.arena.corner_radius - self.rules.arena.top_radius);
                dan = min(dan, dan_to_sphere_inner(
                        point,
                        Vec3::new(o2.x, self.rules.arena.height - self.rules.arena.top_radius, o2.y),
                        self.rules.arena.top_radius));
            }
        }
    }
    

    return dan;
    }



    fn dan_to_arena(&mut self,inPoint: Vec3) -> (f64,Vec3){
        let mut point = inPoint;
        let negate_x = point.x < 0.0;
        let negate_z = point.y < 0.0 ;
        if negate_x {
            point.x = -1.0*point.x;
        }
        if negate_z {
            point.h = -1.0*point.h;
        }

        let mut result = self.dan_to_arena_quarter(point);

        // ye sage rizi zadam sare normal result.normal.x
        if negate_x {
            (result.1).x = -1.0 * (result.1).x;
        }
        if negate_z {
            (result.1).h = -1.0*(result.1).h;
        }
        return result
    }
    // fn collide_with_arena(&mut self) {
    //     let distance, normal = dan_to_arena(e.position)
    //     let penetration = e.radius - distance
    //     if penetration > 0:
    //     e.position += penetration * normal
    //     let velocity = dot(e.velocity, normal) - e.radius_change_speed
    //     if velocity < 0:
    //     e.velocity -= (1 + e.arena_e) * velocity * normal
    //     return normal
    //     return None
    // }
    // fn ballHeightInTheFuture (&mut self,t:f64) -> f64 {
    //    let ballHeight = self.game.ball.height();
    //    let ballhVel = self.game.ball.hVel();

    //    let height = ballhVel * t - 0.5* self.rules.GRAVITY *t *t + ballHeight;

    //    height
    // }
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
