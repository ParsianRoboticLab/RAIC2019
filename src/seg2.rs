#[derive(Copy, Clone, Debug, Default)]
struct Seg2 {
    origin: Vec2,
    terminal: Vec2,
}



impl Seg2 {
    fn new(_origin: Vec2, _terminal: Vec2) -> Self {
        Self {
            origin: _origin,
            terminal: _terminal,
        }
    }

    fn new2(_origin_x: f64, _origin_y: f64, _terminal_x: f64, _terminal_y: f64) -> Self {
        Self {
            origin: Vec2{x:_origin_x, y:_origin_y},
            terminal: Vec2{x:_terminal_x, y:_terminal_y},
        }
    }

    fn new3(_origin: Vec2, _lenght: f64, _dir: AngDeg) -> Self {
        Self {
            origin: _origin,
            terminal: _origin + Vec2::from_polar(_lenght, _dir),
        }
    }

    fn origin(&self) -> Vec2 {
        self.origin
    }

    fn terminal(&self) -> Vec2 {
        self.terminal
    }

    fn length(&self) -> f64 {
        self.origin.dist(self.terminal)
    }

    fn dir(&self) -> AngDeg {
        (self.terminal - self.origin).th()
    }

    fn line(&self) -> Line2 {
        Line2::new2(self.origin, self.terminal)
    }

    fn intersection(&self, other : Seg2) -> Vec2 {
        self.line().intersection(other.line())
    }
}
