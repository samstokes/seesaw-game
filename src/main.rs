extern crate glutin_window;
extern crate graphics;
extern crate nalgebra;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate opengl_graphics;
extern crate piston;

use glutin_window::GlutinWindow as Window;
use graphics::line::Line;
use nalgebra::{Isometry2, Vector2};
use ncollide2d::shape::{Cuboid, Plane, ShapeHandle};
use nphysics2d::{
    object::{BodyHandle, Material},
    world::World,
};
use opengl_graphics::{GlGraphics, OpenGL};
use piston::event_loop;
use piston::input;
use piston::window::WindowSettings;

const G: f64 = 9.81;
const SQUARE_SIZE: f64 = 10.0;
const GROUND_DEPTH: f64 = 150.0;

const PI: f64 = std::f64::consts::PI;

const BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];

const COLLIDER_MARGIN: f64 = 0.01;

pub struct App {
    gl: GlGraphics,
    world: World<f64>,
    squares: Vec<Square>,
    ground: Ground,
    render_count: u16,
}

struct Square {
    size: f64,
    color: [f32; 4],
    physics: BodyHandle,
}

impl Square {
    fn render(
        &self,
        world: &World<f64>,
        transform: &graphics::math::Matrix2d,
        gl: &mut GlGraphics,
    ) {
        use graphics::Transformed;

        let body = world.rigid_body(self.physics).unwrap();
        let trans = body.position().translation.vector;
        let rot = body.position().rotation;

        let square = graphics::rectangle::square(-self.size * 0.5, -self.size * 0.5, self.size);

        let transform = transform.trans(trans[0], trans[1]).rot_rad(rot.angle());

        graphics::rectangle(self.color, square, transform, gl);
    }

    fn new(
        world: &mut World<f64>,
        initial_pos: [f64; 2],
        angle: f64,
        size: f64,
        color: [f32; 4],
    ) -> Self {
        use nphysics2d::volumetric::Volumetric;

        let [x, y] = initial_pos;

        let shape = ShapeHandle::new(Cuboid::new(Vector2::repeat(size * 0.5 - COLLIDER_MARGIN)));
        let physics = world.add_rigid_body(
            Isometry2::new(Vector2::new(x, y), angle),
            shape.inertia(1.0),
            shape.center_of_mass(),
        );

        world.add_collider(
            COLLIDER_MARGIN,
            shape.clone(),
            physics,
            Isometry2::identity(),
            Material::default(),
        );

        Self {
            size: size,
            color: color,
            physics: physics,
        }
    }
}

struct Ground {
    depth: f64,
    line: Line,
}

impl Ground {
    fn new(world: &mut World<f64>, depth: f64, color: [f32; 4]) -> Self {
        let shape = ShapeHandle::new(Plane::new(-Vector2::y_axis()));
        let pos = Isometry2::new(Vector2::y() * depth, nalgebra::zero());
        world.add_collider(
            COLLIDER_MARGIN,
            shape,
            BodyHandle::ground(),
            pos,
            Material::default(),
        );
        Self {
            depth: depth,
            line: Line::new(color, 1.0),
        }
    }

    fn render(&self, transform: &graphics::math::Matrix2d, gl: &mut GlGraphics) {
        let draw_state = graphics::draw_state::DrawState::default();
        self.line.draw(
            [-1_000_000.0, self.depth, 1_000_000.0, self.depth],
            &draw_state,
            *transform,
            gl,
        );
    }
}

fn draw_axes(
    half_width: f64,
    half_height: f64,
    line: &Line,
    transform: graphics::math::Matrix2d,
    gl: &mut GlGraphics,
) {
    let draw_state = graphics::draw_state::DrawState::default();

    line.draw_arrow(
        [-half_width, 0.0, half_width, 0.0],
        5.0,
        &draw_state,
        transform,
        gl,
    );
    line.draw_arrow(
        [0.0, -half_height, 0.0, half_height],
        5.0,
        &draw_state,
        transform,
        gl,
    );
}

impl App {
    fn render(&mut self, args: &input::RenderArgs) {
        use graphics::Transformed;

        self.render_count += 1;
        if self.render_count % 60 == 0 {
            self.render_count = 0;
            println!("{:?}", args);
        }

        let half_width = (args.width / 2) as f64;
        let half_height = (args.height / 2) as f64;

        let ground = &self.ground;
        let squares = &self.squares;
        let world = &self.world;

        self.gl.draw(args.viewport(), |c, gl| {
            graphics::clear(BLACK, gl);

            let white_line = Line::new(WHITE, 1.0);

            // set up so (0, 0) is in the center of the viewport
            let transform = c.transform.trans(half_width, half_height);

            draw_axes(half_width, half_height, &white_line, transform, gl);

            for square in squares {
                square.render(world, &transform, gl);
            }

            ground.render(&transform, gl);
        });
    }

    fn update(&mut self, args: &input::UpdateArgs) {
        self.world.set_timestep(args.dt);
        self.world.step();
    }
}

fn make_world() -> World<f64> {
    let mut world = World::new();

    world.set_gravity(Vector2::y() * G);

    world
}

fn main() {
    let opengl = OpenGL::V3_2;

    let mut window: Window = WindowSettings::new("seesaw", [400, 300])
        .opengl(opengl)
        .exit_on_esc(true)
        .build()
        .unwrap();

    let mut world = make_world();

    let ground = Ground::new(&mut world, GROUND_DEPTH, WHITE);

    let squares = vec![
        Square::new(
            &mut world,
            [-SQUARE_SIZE * 4.5, -SQUARE_SIZE * 5.0],
            0.0,
            SQUARE_SIZE * 0.5,
            RED,
        ),
        Square::new(&mut world, [0.0, 0.0], PI * 0.25 - 0.1, SQUARE_SIZE, RED),
        Square::new(
            &mut world,
            [SQUARE_SIZE * 4.0, 0.0],
            0.0,
            SQUARE_SIZE * 2.0,
            RED,
        ),
        Square::new(
            &mut world,
            [-SQUARE_SIZE * 5.0, 1.0],
            PI * 0.25,
            SQUARE_SIZE * 3.0,
            RED,
        ),
        Square::new(
            &mut world,
            [-SQUARE_SIZE * 10.0, -1.0],
            0.1,
            SQUARE_SIZE * 4.0,
            RED,
        ),
    ];

    let mut app = App {
        gl: GlGraphics::new(opengl),
        world: world,
        squares: squares,
        ground: ground,
        render_count: 0,
    };

    let mut events = event_loop::Events::new(event_loop::EventSettings::new());
    while let Some(e) = events.next(&mut window) {
        use input::RenderEvent;
        use input::UpdateEvent;

        if let Some(r) = e.render_args() {
            app.render(&r);
        }

        if let Some(u) = e.update_args() {
            app.update(&u);
        }
    }
}
