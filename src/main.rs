extern crate glutin_window;
extern crate graphics;
extern crate nalgebra;
extern crate ncollide2d;
extern crate nphysics2d;
extern crate opengl_graphics;
extern crate piston;
extern crate rand;

use glutin_window::GlutinWindow as Window;
use graphics::line::Line;
use nalgebra::{Isometry2, Point2, Unit, Vector2};
use ncollide2d::{
    events::ContactEvent,
    shape::{Ball, ConvexPolygon, Cuboid, Plane, ShapeHandle},
    world::CollisionObjectHandle,
};
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
use std::collections::HashSet;

const G: f64 = 9.81;
const SQUARE_SIZE: f64 = 10.0;
const FULCRUM_HEIGHT: f64 = 10.0;
const PLANK_LENGTH: f64 = 80.0;
const PLANK_THICKNESS: f64 = 2.0;
const GROUND_DEPTH: f64 = 150.0;

const PI: f64 = std::f64::consts::PI;

const BLACK: [f32; 4] = [0.0, 0.0, 0.0, 1.0];
const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
const SEMIGREY: [f32; 4] = [0.5, 0.5, 0.5, 0.5];
const RED: [f32; 4] = [1.0, 0.0, 0.0, 1.0];
const BLUE: [f32; 4] = [0.0, 0.0, 1.0, 1.0];
const GREEN: [f32; 4] = [0.0, 1.0, 0.0, 1.0];

fn jitter(base: [f32; 4], delta: [f32; 4]) -> [f32; 4] {
    let mut result: [f32; 4] = [0.0; 4];
    for i in 0..4 {
        result[i] = base[i] + rand::random::<f32>() * delta[i];
    }
    result
}

const COLLIDER_MARGIN: f64 = 0.01;

pub struct App {
    gl: GlGraphics,
    world: World<f64>,
    player: Player,
    objects: Vec<Box<Object>>,
    squares: Vec<Square>,
    seesaws: Vec<Seesaw>,
    ground: Ground,
    render_count: u16,
    debug: bool,
}

struct Player {
    body: Square,
    contacts: HashSet<CollisionObjectHandle>,
    impulse_up: Option<ForceGeneratorHandle>,
    impulse_left: Option<ForceGeneratorHandle>,
    impulse_right: Option<ForceGeneratorHandle>,
}

impl Player {
    fn render(
        &self,
        world: &World<f64>,
        transform: &graphics::math::Matrix2d,
        gl: &mut GlGraphics,
        debug: bool,
    ) {
        self.body.render(world, transform, gl, debug);
    }

    fn new(world: &mut World<f64>, initial_pos: [f64; 2]) -> Self {
        Self {
            body: Square::new(world, initial_pos, 0.0, 10.0, GREEN),
            contacts: HashSet::new(),
            impulse_up: None,
            impulse_left: None,
            impulse_right: None,
        }
    }

    fn handle(&self) -> BodyHandle {
        self.body.physics
    }

    fn collision_handle(&self) -> CollisionObjectHandle {
        self.body.collision
    }

    fn can_jump(&self) -> bool {
        // TODO actually check we are _above_ the contacts
        // or generalise that so we can do wall-jumps etc?
        !self.contacts.is_empty()
    }

    fn move_up(&mut self, world: &mut World<f64>) {
        if self.impulse_up.is_some() || !self.can_jump() {
            return;
        };
        let impulse = KeyboardImpulse::new(-Vector2::y_axis(), self.handle());
        self.impulse_up = Some(world.add_force_generator(impulse));
    }

    fn stop_moving_up(&mut self, world: &mut World<f64>) {
        for impulse in self.impulse_up {
            world.remove_force_generator(impulse);
            self.impulse_up = None;
        }
    }

    fn move_left(&mut self, world: &mut World<f64>) {
        if self.impulse_left.is_some() {
            return;
        };
        let impulse = KeyboardImpulse::new(-Vector2::x_axis(), self.handle());
        self.impulse_left = Some(world.add_force_generator(impulse));
    }

    fn stop_moving_left(&mut self, world: &mut World<f64>) {
        for impulse in self.impulse_left {
            world.remove_force_generator(impulse);
            self.impulse_left = None;
        }
    }

    fn move_right(&mut self, world: &mut World<f64>) {
        if self.impulse_right.is_some() {
            return;
        };
        let impulse = KeyboardImpulse::new(Vector2::x_axis(), self.handle());
        self.impulse_right = Some(world.add_force_generator(impulse));
    }

    fn stop_moving_right(&mut self, world: &mut World<f64>) {
        for impulse in self.impulse_right {
            world.remove_force_generator(impulse);
            self.impulse_right = None;
        }
    }
}

struct Square {
    size: f64,
    color: [f32; 4],
    physics: BodyHandle,
    collision: CollisionObjectHandle,
}

impl Square {
    fn render(
        &self,
        world: &World<f64>,
        transform: &graphics::math::Matrix2d,
        gl: &mut GlGraphics,
        debug: bool,
    ) {
        use graphics::Transformed;

        let part = world.body_part(self.physics);
        let trans = part.position().translation.vector;
        let rot = part.position().rotation;

        let square = graphics::rectangle::square(-self.size * 0.5, -self.size * 0.5, self.size);

        let transform = transform.trans(trans[0], trans[1]);

        if debug {
            let velocity = part.velocity().linear;
            let speed = velocity.magnitude();
            let line = Line::new(SEMIGREY, 0.5);
            let draw_state = graphics::draw_state::DrawState::default();
            line.draw_arrow(
                [0.0, 0.0, velocity[0], velocity[1]],
                speed * 0.2,
                &draw_state,
                transform,
                gl,
            );
        }

        let transform = transform.rot_rad(rot.angle());

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

        let collision = world.add_collider(
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
            collision: collision,
        }
    }
}

impl Object for Square {
    fn physics(&self) -> BodyHandle {
        self.physics
    }

    fn render_in_own_frame(&self, transform: graphics::math::Matrix2d, gl: &mut GlGraphics) {
        let square = graphics::rectangle::square(-self.size * 0.5, -self.size * 0.5, self.size);

        graphics::rectangle(self.color, square, transform, gl);
    }
}

struct Circle {
    size: f64,
    color: [f32; 4],
    physics: BodyHandle,
}

impl Circle {
    fn new(world: &mut World<f64>, initial_pos: [f64; 2], size: f64, color: [f32; 4]) -> Self {
        use nphysics2d::volumetric::Volumetric;

        let [x, y] = initial_pos;

        let shape = ShapeHandle::new(Ball::new(size * 0.5 - COLLIDER_MARGIN));
        let physics = world.add_rigid_body(
            Isometry2::new(Vector2::new(x, y), 0.0),
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

impl Object for Circle {
    fn physics(&self) -> BodyHandle {
        self.physics
    }

    fn render_in_own_frame(&self, transform: graphics::math::Matrix2d, gl: &mut GlGraphics) {
        let circle = graphics::ellipse::circle(0.0, 0.0, self.size * 0.5);

        graphics::ellipse(self.color, circle, transform, gl);
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

        let part = world.body_part(self.physics);
        let trans = part.position().translation.vector;
        let rot = part.position().rotation;

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

        let part = world.body_part(self.physics);
        let trans = part.position().translation.vector;
        let rot = part.position().rotation;

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

trait Object {
    fn physics(&self) -> BodyHandle;
    fn render_in_own_frame(&self, transform: graphics::math::Matrix2d, gl: &mut GlGraphics);
}

impl Object {
    fn render(
        &self,
        world: &World<f64>,
        transform: &graphics::math::Matrix2d,
        gl: &mut GlGraphics,
        debug: bool,
    ) {
        use graphics::Transformed;

        let part = world.body_part(self.physics());
        let position = part.position();
        let trans = position.translation.vector;
        let rot = position.rotation;

        let transform = transform.trans(trans[0], trans[1]);

        if debug {
            let velocity = part.velocity().linear;
            let speed = velocity.magnitude();
            let line = Line::new(SEMIGREY, 0.5);
            let draw_state = graphics::draw_state::DrawState::default();
            line.draw_arrow(
                [0.0, 0.0, velocity[0], velocity[1]],
                speed * 0.2,
                &draw_state,
                transform,
                gl,
            );
        }

        let transform = transform.rot_rad(rot.angle());

        self.render_in_own_frame(transform, gl);
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
        const MAGNITUDE: f64 = 15000.0;
        const MAX_SPEED: f64 = 50.0;

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
    transform: graphics::math::Matrix2d,
    gl: &mut GlGraphics,
) {
    let line = Line::new(SEMIGREY, 1.0);

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

        if self.debug {
            self.render_count += 1;
            if self.render_count % 60 == 0 {
                self.render_count = 0;
                println!("{:?}", args);
            }
        }

        let half_width = (args.width / 2) as f64;
        let half_height = (args.height / 2) as f64;

        let ground = &self.ground;
        let player = &self.player;
        let objects = &self.objects;
        let squares = &self.squares;
        let seesaws = &self.seesaws;
        let world = &self.world;
        let debug = self.debug;

        self.gl.draw(args.viewport(), |c, gl| {
            graphics::clear(BLACK, gl);

            // set up so (0, 0) is in the center of the viewport
            let transform = c.transform.trans(half_width, half_height);

            if debug {
                draw_axes(half_width, half_height, transform, gl);
            }

            player.render(world, &transform, gl, debug);

            for object in objects {
                object.as_ref().render(world, &transform, gl, debug);
            }

            for square in squares {
                square.render(world, &transform, gl, debug);
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

        for event in self.world.contact_events() {
            match event {
                ContactEvent::Started(collision1, collision2) => {
                    if *collision1 == self.player.collision_handle() {
                        self.player.contacts.insert(*collision2);
                    }
                    if *collision2 == self.player.collision_handle() {
                        self.player.contacts.insert(*collision1);
                    }
                }
                ContactEvent::Stopped(collision1, collision2) => {
                    if *collision1 == self.player.collision_handle() {
                        self.player.contacts.remove(collision2);
                    }
                    if *collision2 == self.player.collision_handle() {
                        self.player.contacts.remove(collision1);
                    }
                }
            }
        }
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

    let squares = vec![];

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

    let objects: Vec<Box<Object>> = vec![
        // Squares
        Box::new(Square::new(
            &mut world,
            [-SQUARE_SIZE * 4.5, -SQUARE_SIZE * 5.0],
            0.0,
            SQUARE_SIZE * 0.5,
            jitter(RED, [-0.4, 0.0, 0.0, 0.0]),
        )),
        Box::new(Square::new(
            &mut world,
            [0.0, 0.0],
            PI * 0.25 - 0.1,
            SQUARE_SIZE,
            jitter(RED, [-0.4, 0.0, 0.0, 0.0]),
        )),
        Box::new(Square::new(
            &mut world,
            [-SQUARE_SIZE * 5.0, 1.0],
            PI * 0.25,
            SQUARE_SIZE * 2.0,
            jitter(RED, [-0.4, 0.0, 0.0, 0.0]),
        )),
        Box::new(Square::new(
            &mut world,
            [SQUARE_SIZE * 5.0, -50.0],
            0.0,
            SQUARE_SIZE * 3.0,
            jitter(RED, [-0.4, 0.0, 0.0, 0.0]),
        )),
        Box::new(Square::new(
            &mut world,
            [-SQUARE_SIZE * 10.0, -1.0],
            0.1,
            SQUARE_SIZE * 4.0,
            jitter(RED, [-0.4, 0.0, 0.0, 0.0]),
        )),
        // Circles
        Box::new(Circle::new(
            &mut world,
            [-SQUARE_SIZE * 10.0, -20.0],
            SQUARE_SIZE * 2.0,
            jitter(RED, [-0.4, 0.0, 0.0, 0.0]),
        )),
    ];

    let mut app = App {
        gl: GlGraphics::new(opengl),
        world: world,
        player: player,
        objects: objects,
        squares: squares,
        seesaws: seesaws,
        ground: ground,
        render_count: 0,
        debug: debug,
    };

    let mut events = event_loop::Events::new(event_loop::EventSettings::new());
    while let Some(e) = events.next(&mut window) {
        use input::PressEvent;
        use input::ReleaseEvent;
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
            match key {
                H | Left => app.player.move_left(&mut app.world),
                K | Up | Space => app.player.move_up(&mut app.world),
                L | Right => app.player.move_right(&mut app.world),
                _ => (),
            }
        }

        if let Some(input::Button::Keyboard(key)) = e.release_args() {
            use input::Key::*;
            match key {
                H | Left => app.player.stop_moving_left(&mut app.world),
                K | Up | Space => app.player.stop_moving_up(&mut app.world),
                L | Right => app.player.stop_moving_right(&mut app.world),
                _ => (),
            }
        }
    }
}
