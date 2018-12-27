use crate::model::*;
use crate::strategy::Strategy;

impl Default for Arena {
    fn default() -> Self {
        Self {
            width: 0.,
            height: 0.,
            depth: 0.,
            bottom_radius: 0.,
            top_radius: 0.,
            corner_radius: 0.,
            goal_top_radius: 0.,
            goal_width: 0.,
            goal_height: 0.,
            goal_depth: 0.,
            goal_side_radius: 0.,
        }
    }
}

impl Default for Rules {
    fn default() -> Self {
        Self {
            max_tick_count: 0,
            arena: Arena{..Default::default()},
            team_size: 0,
            seed: 0,
            ROBOT_MIN_RADIUS: 0.,
            ROBOT_MAX_RADIUS: 0.,
            ROBOT_MAX_JUMP_SPEED: 0.,
            ROBOT_ACCELERATION: 0.,
            ROBOT_NITRO_ACCELERATION: 0.,
            ROBOT_MAX_GROUND_SPEED: 0.,
            ROBOT_ARENA_E: 0.,
            ROBOT_RADIUS: 0.,
            ROBOT_MASS: 0.,
            TICKS_PER_SECOND: 0,
            MICROTICKS_PER_TICK: 0,
            RESET_TICKS: 0,
            BALL_ARENA_E: 0.,
            BALL_RADIUS: 0.,
            BALL_MASS: 0.,
            MIN_HIT_E: 0.,
            MAX_HIT_E: 0.,
            MAX_ENTITY_SPEED: 0.,
            MAX_NITRO_AMOUNT: 0.,
            START_NITRO_AMOUNT: 0.,
            NITRO_POINT_VELOCITY_CHANGE: 0.,
            NITRO_PACK_X: 0.,
            NITRO_PACK_Y: 0.,
            NITRO_PACK_Z: 0.,
            NITRO_PACK_RADIUS: 0.,
            NITRO_PACK_AMOUNT: 0.,
            NITRO_PACK_RESPAWN_TICKS: 0,
            GRAVITY: 0., 
        }
    }
}


impl Default for Robot {
    fn default() -> Self {
        Self {
           id: 0,
           player_id: 0,
           is_teammate: false,
           x: 0.0,
           y: 0.0,
           z: 0.0,
           velocity_x: 0.0,
           velocity_y: 0.0,
           velocity_z: 0.0,
           radius: 0.0,
           nitro_amount: 0.0,
           touch: false,
           touch_normal_x: Some(0.0),
           touch_normal_y: Some(0.0),
           touch_normal_z: Some(0.0),
        }
    }
}

impl Default for Ball {
    fn default() -> Self {
    Self{
        x: 0.0,          
        y: 0.0,
        z: 0.0,
        velocity_x: 0.0,
        velocity_y: 0.0,
        velocity_z: 0.0,
        radius: 0.0,
    }
    }
}

impl Default for Game {
    fn default() -> Self {
        Self{
            current_tick: 0,
            players: vec![],
            robots: Vec::new(),
            nitro_packs: Vec::new(),
            ball: Ball{..Default::default()}
        }
    }
}


struct PID {
    kp : f64,
    ki : f64,
    kd : f64,
    sum : f64,
    last: f64,
}

impl Default for PID {
    fn default() -> Self {
        Self{
            kp:0.0,
            ki:0.0,
            kd:0.0,
            sum:0.0,
            last:0.0,
        }
    }
}

impl PID {
    fn run(&mut self, err: f64) -> f64 {
        self.sum += err;
        let res = (self.kp * err) + (self.ki * self.sum) + (self.kd * (err - self.last));
        self.last = err;
        res
    }
}

// We will need to work with 2d vertors
#[derive(Copy, Clone, Debug)]
struct Vec2 {
    x: f64,
    y: f64,
}

impl Vec2 {
    fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    // Finding length of the vector
    fn len(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    // Normalizing vector (setting its length to 1)
    fn normalize(self) -> Self {
        self * (1.0 / self.len())
    }

    fn dist(&self, other: Self) -> f64 {
        (*self - other).len()
    }
}

// Subtraction operation for vectors
impl std::ops::Sub for Vec2 {
    type Output = Self;
    fn sub(self, b: Self) -> Self {
        Self::new(self.x - b.x, self.y - b.y)
    }
}

// Addition for vectors
impl std::ops::Add for Vec2 {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self::new(self.x + b.x, self.y + b.y)
    }
}

// Multiplying vector by a number
impl std::ops::Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, k: f64) -> Self {
        Self::new(self.x * k, self.y * k)
    }
}

// Common interface for ball and robots
trait Entity {
    // Position of the entity in XZ plane
    fn position(&self) -> Vec2;
    // Velocity of the entity in XZ plane
    fn velocity(&self) -> Vec2;
    // Height
    fn height(&self) -> f64;
}

impl Entity for Robot {
    fn position(&self) -> Vec2 {
        Vec2::new(self.x, self.z)
    }
    fn velocity(&self) -> Vec2 {
        Vec2::new(self.velocity_x, self.velocity_z)
    }
    fn height(&self) -> f64 {
        self.y
    }
}

impl Entity for Ball {
    fn position(&self) -> Vec2 {
        Vec2::new(self.x, self.z)
    }
    fn velocity(&self) -> Vec2 {
        Vec2::new(self.velocity_x, self.velocity_z)
    }
    fn height(&self) -> f64 {
        self.y
    }
}

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
        for robot in &_game.robots {
            if robot.is_teammate && robot.id != me.id {
                if robot.position().y < me.position().y {
                     return Role::OFF
                } else {
                    return Role::GK
                }
            }
        }
        Role::NONE
    }
}

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
            posPID: PID{
                kp:15.0,
                ki:0.0,
                kd:0.0,
                sum:0.0,
                last:0.0
            }
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
