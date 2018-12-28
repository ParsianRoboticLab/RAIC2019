use super::*;

#[allow(non_snake_case)]
#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Rules {
    pub max_tick_count: i32,
    pub arena: Arena,
    pub team_size: i32,
    pub seed: i64,
    pub ROBOT_MIN_RADIUS: f64,
    pub ROBOT_MAX_RADIUS: f64,
    pub ROBOT_MAX_JUMP_SPEED: f64,
    pub ROBOT_ACCELERATION: f64,
    pub ROBOT_NITRO_ACCELERATION: f64,
    pub ROBOT_MAX_GROUND_SPEED: f64,
    pub ROBOT_ARENA_E: f64,
    pub ROBOT_RADIUS: f64,
    pub ROBOT_MASS: f64,
    pub TICKS_PER_SECOND: usize,
    pub MICROTICKS_PER_TICK: usize,
    pub RESET_TICKS: usize,
    pub BALL_ARENA_E: f64,
    pub BALL_RADIUS: f64,
    pub BALL_MASS: f64,
    pub MIN_HIT_E: f64,
    pub MAX_HIT_E: f64,
    pub MAX_ENTITY_SPEED: f64,
    pub MAX_NITRO_AMOUNT: f64,
    pub START_NITRO_AMOUNT: f64,
    pub NITRO_POINT_VELOCITY_CHANGE: f64,
    pub NITRO_PACK_X: f64,
    pub NITRO_PACK_Y: f64,
    pub NITRO_PACK_Z: f64,
    pub NITRO_PACK_RADIUS: f64,
    pub NITRO_PACK_AMOUNT: f64,
    pub NITRO_PACK_RESPAWN_TICKS: usize,
    pub GRAVITY: f64,
}