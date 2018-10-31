extern crate glutin_window;
extern crate graphics;
extern crate nalgebra;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate opengl_graphics;
extern crate piston;

use glutin_window::GlutinWindow as Window;
use graphics::line::Line;
use nalgebra::{Isometry2, Point2, Unit, Vector2};
use ncollide2d::shape::{ConvexPolygon, Cuboid, Plane, ShapeHandle};
use nphysics2d::{
    force_generator::{ForceGenerator, ForceGeneratorHandle},
    joint::{FreeJoint, RevoluteJoint},
    math::Force,
    object::{BodyHandle, BodySet, Material},
    solver::IntegrationParameters,
    world::World,
};
use opengl_graphics::{GlGraphics, OpenGL};
use piston::event_loop;
use piston::input;
use piston::window::WindowSettings;

const G: f64 = 9.81;
const SQUARE_SIZE: f64 = 10.0;
const FULCRUM_HEIGHT: f64 = 10.0;
const PLANK_LENGTH: f64 = 80.0;
const PLANK_THICKNESS: f64 = 2.0;
const GROUND_DEPTH: f64 = 150.0;

const PI: f64 = std::f64::consts::PI;

const BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
const BLUE: [f32; 4] = [0.0, 0.0, 1.0, 1.0];
const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];

const COLLIDER_MARGIN: f64 = 0.01;

pub struct App {
    gl: GlGraphics,
    world: World<f64>,
    player: Player,
    squares: Vec<Square>,
    seesaws: Vec<Seesaw>,
    ground: Ground,
    render_count: u16,
    debug: bool,
}

struct Player {
    body: Square,
    impulse: Option<ForceGeneratorHandle>,
}

impl Player {
    fn render(
        &self,
        world: &World<f64>,
        transform: &graphics::math::Matrix2d,
        gl: &mut GlGraphics,
    ) {
        self.body.render(world, transform, gl);
    }

    fn new(world: &mut World<f64>, initial_pos: [f64; 2]) -> Self {
        Self {
            body: Square::new(world, initial_pos, 0.0, 10.0, GREEN),
            impulse: None,
        }
    }

    fn handle(&self) -> BodyHandle {
        self.body.physics
    }
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
        initial_angle: f64,
        size: f64,
        color: [f32; 4],
    ) -> Self {
        use nphysics2d::volumetric::Volumetric;

        let [x, y] = initial_pos;

        let shape = ShapeHandle::new(Cuboid::new(Vector2::repeat(size * 0.5 - COLLIDER_MARGIN)));
        let physics = world.add_rigid_body(
            Isometry2::new(Vector2::new(x, y), initial_angle),
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
        const LINE_WIDTH: f64 = 1.0;

        let shape = ShapeHandle::new(Plane::new(-Vector2::y_axis()));
        let pos = Isometry2::new(
            Vector2::y() * (depth - LINE_WIDTH + COLLIDER_MARGIN),
            nalgebra::zero(),
        );
        world.add_collider(
            COLLIDER_MARGIN,
            shape,
            BodyHandle::ground(),
            pos,
            Material::default(),
        );
        Self {
            depth: depth,
            line: Line::new(color, LINE_WIDTH),
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

struct Seesaw {
    fulcrum: Fulcrum,
    plank: Plank,
}

impl Seesaw {
    fn render(
        &self,
        world: &World<f64>,
        transform: &graphics::math::Matrix2d,
        gl: &mut GlGraphics,
    ) {
        self.fulcrum.render(world, transform, gl);
        self.plank.render(world, transform, gl);
    }

    fn new(
        world: &mut World<f64>,
        initial_x: f64,
        initial_angle: f64,
        height: f64,
        length: f64,
        thickness: f64,
        offset: f64,
        color: [f32; 4],
    ) -> Self {
        let fulcrum = Fulcrum::new(world, initial_x, height, color);
        let plank = Plank::new(
            world,
            &fulcrum,
            initial_angle,
            length,
            thickness,
            offset,
            color,
        );
        Self {
            fulcrum: fulcrum,
            plank: plank,
        }
    }
}

struct Fulcrum {
    height: f64,
    color: [f32; 4],
    physics: BodyHandle,
}

impl Fulcrum {
    fn render(
        &self,
        world: &World<f64>,
        transform: &graphics::math::Matrix2d,
        gl: &mut GlGraphics,
    ) {
        use graphics::Transformed;

        let body = world.multibody_link(self.physics).unwrap();
        let trans = body.position().translation.vector;
        let rot = body.position().rotation;

        let half_height = self.height * 0.5;
        let polygon = [
            [-half_height, half_height],
            [0.0, -half_height],
            [half_height, half_height],
        ];

        let transform = transform.trans(trans[0], trans[1]).rot_rad(rot.angle());

        graphics::polygon(self.color, &polygon, transform, gl);
    }

    fn new(world: &mut World<f64>, initial_x: f64, height: f64, color: [f32; 4]) -> Self {
        use nphysics2d::volumetric::Volumetric;

        let half_height = height * 0.5;

        let shape = ShapeHandle::new(
            ConvexPolygon::try_from_points(&[
                Point2::new(
                    -half_height + COLLIDER_MARGIN,
                    half_height - COLLIDER_MARGIN,
                ),
                Point2::new(0.0, -half_height + COLLIDER_MARGIN),
                Point2::new(half_height - COLLIDER_MARGIN, half_height - COLLIDER_MARGIN),
            ])
            .unwrap(),
        );
        let physics = world.add_multibody_link(
            BodyHandle::ground(),
            /* TODO should be this FixedJoint, but it panics in update_inertias
            FixedJoint::new(Isometry2::new(
                -Vector2::new(initial_x, GROUND_DEPTH - half_height),
                nalgebra::zero(),
            )),
            */
            FreeJoint::new(Isometry2::new(
                Vector2::new(initial_x, GROUND_DEPTH - half_height),
                nalgebra::zero(),
            )),
            nalgebra::zero(),
            nalgebra::zero(),
            shape.inertia(10.0),
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
            height: height,
            color: color,
            physics: physics,
        }
    }
}

struct Plank {
    length: f64,
    thickness: f64,
    color: [f32; 4],
    physics: BodyHandle,
}

impl Plank {
    fn render(
        &self,
        world: &World<f64>,
        transform: &graphics::math::Matrix2d,
        gl: &mut GlGraphics,
    ) {
        use graphics::Transformed;

        let body = world.multibody_link(self.physics).unwrap();
        let trans = body.position().translation.vector;
        let rot = body.position().rotation;

        let rectangle =
            graphics::rectangle::centered([0.0, 0.0, self.length * 0.5, self.thickness * 0.5]);

        let transform = transform.trans(trans[0], trans[1]).rot_rad(rot.angle());

        graphics::rectangle(self.color, rectangle, transform, gl);
    }

    fn new(
        world: &mut World<f64>,
        fulcrum: &Fulcrum,
        initial_angle: f64,
        length: f64,
        thickness: f64,
        offset: f64,
        color: [f32; 4],
    ) -> Self {
        use nphysics2d::volumetric::Volumetric;

        let half_length = length * 0.5;
        let half_thickness = thickness * 0.5;

        assert!(offset.abs() < half_length);

        let shape = ShapeHandle::new(Cuboid::new(Vector2::new(
            half_length - COLLIDER_MARGIN,
            half_thickness - COLLIDER_MARGIN,
        )));

        let mut joint = RevoluteJoint::new(initial_angle);
        joint.enable_min_angle(-PI / 4.0);
        joint.enable_max_angle(PI / 4.0);

        let physics = world.add_multibody_link(
            fulcrum.physics,
            joint,
            Vector2::new(0.0, -fulcrum.height * 0.5),
            Vector2::new(offset, half_thickness),
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
            length: length,
            thickness: thickness,
            color: color,
            physics: physics,
        }
    }
}

struct KeyboardImpulse {
    direction: Unit<Vector2<f64>>,
    target: BodyHandle,
}

impl KeyboardImpulse {
    fn new(direction: Unit<Vector2<f64>>, target: BodyHandle) -> Self {
        Self {
            direction: direction,
            target: target,
        }
    }
}

impl ForceGenerator<f64> for KeyboardImpulse {
    fn apply(&mut self, _params: &IntegrationParameters<f64>, bodies: &mut BodySet<f64>) -> bool {
        const MAGNITUDE: f64 = 500000.0;
        const MAX_SPEED: f64 = 25.0;

        let mut target = bodies.body_part_mut(self.target);

        if target.as_ref().velocity().linear.dot(&self.direction) < MAX_SPEED {
            target.apply_force(&Force::linear(self.direction.as_ref() * MAGNITUDE));
        }

        false // docs say false will remove this ForceGenerator after application, but it is actually ignored
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
        let player = &self.player;
        let squares = &self.squares;
        let seesaws = &self.seesaws;
        let world = &self.world;
        let debug = self.debug;

        self.gl.draw(args.viewport(), |c, gl| {
            graphics::clear(BLACK, gl);

            let white_line = Line::new(WHITE, 1.0);

            // set up so (0, 0) is in the center of the viewport
            let transform = c.transform.trans(half_width, half_height);

            if debug {
                draw_axes(half_width, half_height, &white_line, transform, gl);
            }

            player.render(world, &transform, gl);

            for square in squares {
                square.render(world, &transform, gl);
            }

            for seesaw in seesaws {
                seesaw.render(world, &transform, gl);
            }

            ground.render(&transform, gl);
        });
    }

    fn update(&mut self, args: &input::UpdateArgs) {
        self.world.set_timestep(args.dt);
        self.world.step();
        for impulse in self.player.impulse {
            self.world.remove_force_generator(impulse);
            self.player.impulse = None;
        }
    }

    fn move_player(&mut self, direction: Unit<Vector2<f64>>) {
        for impulse in self.player.impulse {
            self.world.remove_force_generator(impulse);
            self.player.impulse = None;
        }
        let impulse = KeyboardImpulse::new(direction, self.player.handle());
        self.player.impulse = Some(self.world.add_force_generator(impulse));
    }
}

fn make_world() -> World<f64> {
    let mut world = World::new();

    world.set_gravity(Vector2::y() * G);

    world
}

fn main() {
    let debug = std::env::args()
        .nth(1)
        .filter(|arg| arg == "--debug")
        .is_some();

    let opengl = OpenGL::V3_2;

    let mut window: Window = WindowSettings::new("seesaw", [400, 300])
        .opengl(opengl)
        .exit_on_esc(true)
        .build()
        .unwrap();

    let mut world = make_world();

    let ground = Ground::new(&mut world, GROUND_DEPTH, WHITE);

    let player = Player::new(&mut world, [-SQUARE_SIZE * 10.0, GROUND_DEPTH - 10.0]);

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
            [-SQUARE_SIZE * 5.0, 1.0],
            PI * 0.25,
            SQUARE_SIZE * 2.0,
            RED,
        ),
        Square::new(
            &mut world,
            [SQUARE_SIZE * 5.0, -50.0],
            0.0,
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

    let seesaws = vec![
        Seesaw::new(
            &mut world,
            30.0,
            -PI / 8.0,
            FULCRUM_HEIGHT,
            PLANK_LENGTH,
            PLANK_THICKNESS,
            PLANK_LENGTH / 4.0,
            BLUE,
        ),
        Seesaw::new(
            &mut world,
            30.0 + PLANK_LENGTH / 2.0 - 1.0,
            PI / 8.0,
            FULCRUM_HEIGHT / 2.0,
            PLANK_LENGTH / 2.0,
            PLANK_THICKNESS,
            -PLANK_LENGTH / 8.0,
            BLUE,
        ),
    ];

    let mut app = App {
        gl: GlGraphics::new(opengl),
        world: world,
        player: player,
        squares: squares,
        seesaws: seesaws,
        ground: ground,
        render_count: 0,
        debug: debug,
    };

    let mut events = event_loop::Events::new(event_loop::EventSettings::new());
    while let Some(e) = events.next(&mut window) {
        use input::PressEvent;
        use input::RenderEvent;
        use input::UpdateEvent;

        if let Some(r) = e.render_args() {
            app.render(&r);
        }

        if let Some(u) = e.update_args() {
            app.update(&u);
        }

        if let Some(input::Button::Keyboard(key)) = e.press_args() {
            use input::Key::*;
            if let Some(direction) = match key {
                H | Left => Some(-Vector2::x_axis()),
                J | Down => Some(Vector2::y_axis()),
                K | Up | Space => Some(-Vector2::y_axis()),
                L | Right => Some(Vector2::x_axis()),
                _ => None,
            } {
                app.move_player(direction)
            }
        }
    }
}
