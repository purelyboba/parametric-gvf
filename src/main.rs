use nannou::prelude::*;
mod model;
mod update;
mod view;
mod types;
mod utils;

use model::create_model;
use update::update;
use view::view;

fn main() {
    nannou::app(create_model)
        .update(update)
        .run();
}