#[derive(Debug, Serialize, Deserialize, Copy, Clone, Default)]
struct PID {
    kp : f64,
    ki : f64,
    kd : f64,
    sum : f64,
    last: f64,
}

impl PID {
    fn new(_kp: f64, _ki: f64, _kd: f64) -> Self {
        Self {
            kp: _kp,
            ki: _ki,
            kd: _kd,
            ..Default::default()
        }
    }
    fn run(&mut self, err: f64) -> f64 {
        self.sum += err;
        let res = (self.kp * err) + (self.ki * self.sum) + (self.kd * (err - self.last));
        self.last = err;
        res
    }
}
