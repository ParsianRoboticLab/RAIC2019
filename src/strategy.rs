use crate::model::*;

pub trait Strategy {
    fn act(&mut self, me: &Robot, rules: &Rules, game: &Game, action: &mut Action);
    fn custom_rendering(&mut self) -> String {
        String::new()
    }
}
