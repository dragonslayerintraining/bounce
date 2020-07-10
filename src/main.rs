use ggez::{graphics, Context, ContextBuilder, GameResult};
use ggez::event::{self,EventHandler};
use ggez::input::mouse::MouseButton;

fn main() -> GameResult{
    let (mut ctx, mut event_loop) = ContextBuilder::new("bounce","dragonslayerintraining")
        .window_setup(ggez::conf::WindowSetup::default().title("Bounce"))
        .window_mode(ggez::conf::WindowMode::default().dimensions(500.0,500.0))
        .build()?;
    let mut world = World::new(&mut ctx);
    event::run(&mut ctx,&mut event_loop,&mut world)
}

#[derive(Clone)]
struct Ball{
    x: f32,
    y: f32,
    xdir: f32,
    ydir: f32,
}

struct World{
    balls: Vec<Ball>,
    ghost: Option<Ball>,
}

enum PhysicsEvent{
    NoEvent,
    BallHWallCollision(usize),
    BallVWallCollision(usize),
    BallBallCollision(usize,usize),
}

impl World{
    fn new(_ctx:&mut Context) -> World{
        World{
            balls: vec![],
            ghost: None,
        }
    }
    fn simple_update(&mut self, dt: f32) {
        for ball in &mut self.balls {
            ball.x+=ball.xdir*dt;
            ball.y+=ball.ydir*dt;
        }
    }
    fn update(&mut self, step: f32) {
        let mut dt = step;
        loop {
            let mut til: f32 = dt;
            let mut ev: PhysicsEvent = PhysicsEvent::NoEvent;
            for (i, ball) in self.balls.iter().enumerate() {
                if ball.xdir < 0.0 {
                    let t = (20.0-ball.x)/ball.xdir;
                    if til > t { til=t; ev = PhysicsEvent::BallHWallCollision(i); }
                }
                if ball.xdir > 0.0 {
                    let t = (500.0-20.0-ball.x)/ball.xdir;
                    if til > t { til=t; ev = PhysicsEvent::BallHWallCollision(i); }
                }
                if ball.ydir < 0.0 {
                    let t = (20.0-ball.y)/ball.ydir;
                    if til > t { til=t; ev = PhysicsEvent::BallVWallCollision(i); }
                }
                if ball.ydir > 0.0 {
                    let t = (500.0-20.0-ball.y)/ball.ydir;
                    if til > t { til=t; ev = PhysicsEvent::BallVWallCollision(i); }
                }
            }
            for (i, ball1) in self.balls.iter().enumerate() {
                for (j, ball2) in self.balls.iter().enumerate() {
                    let relx = ball2.x - ball1.x;        
                    let rely = ball2.y - ball1.y;        
                    let relxdir = ball2.xdir - ball1.xdir;        
                    let relydir = ball2.ydir - ball1.ydir;      
                    let a = relxdir*relxdir+relydir*relydir;
                    let b = 2.0*(relxdir*relx+relydir*rely);
                    let c = relx*relx+rely*rely-(20.0+20.0)*(20.0+20.0);
                    let discr = b*b-4.0*a*c;
                    if discr < 0.0 {continue;}
                    let t1 = (-b-discr.sqrt())/2.0/a;
                    let t2 = (-b+discr.sqrt())/2.0/a;
                    if t1 < 0.0 && t2 > 0.0 { panic!("Overlapping balls!"); }
                    if t2 < 0.0 {continue;}
                    if til > t1 { til=t1; ev = PhysicsEvent::BallBallCollision(i,j); }
                }
            }
            self.simple_update(til);
            dt-=til;
            match ev {
                PhysicsEvent::NoEvent => break,
                PhysicsEvent::BallHWallCollision(i) => self.balls[i].xdir *= -1.0,
                PhysicsEvent::BallVWallCollision(i) => self.balls[i].ydir *= -1.0,
                PhysicsEvent::BallBallCollision(i,j) => {
                    self.balls[i].xdir=0.0;
                    self.balls[i].ydir=0.0;
                    self.balls[j].xdir=0.0;
                    self.balls[j].ydir=0.0;
                }

            }
        }
    }
}

impl EventHandler for World{
    fn update(&mut self,_ctx:&mut Context) -> GameResult{
        self.update(1.0);
        Ok(())
    }
    fn draw(&mut self,ctx:&mut Context) -> GameResult{
        graphics::clear(ctx,graphics::WHITE);
        let circle = graphics::Mesh::new_circle(ctx,graphics::DrawMode::fill(),ggez::nalgebra::Point2::new(0.0,0.0),20.0,0.1,graphics::BLACK)?;
        for ball in &self.balls {
            graphics::draw(ctx,&circle,(ggez::nalgebra::Point2::new(ball.x,ball.y),))?;
        }
        let ghost_circle = graphics::Mesh::new_circle(ctx,graphics::DrawMode::fill(),ggez::nalgebra::Point2::new(0.0,0.0),20.0,2.0,[0.0,0.0,0.0,0.5].into())?;
        if let Some(ball) = &self.ghost {
            graphics::draw(ctx,&ghost_circle,(ggez::nalgebra::Point2::new(ball.x,ball.y),))?;
        }
        graphics::present(ctx)?;
        Ok(())
    }
    fn mouse_button_down_event(&mut self, _ctx: &mut Context,button: MouseButton, x: f32, y: f32){
        match button {
            MouseButton::Left => self.ghost = Some(Ball{x:x,y:y,xdir:0.0,ydir:0.0}),
            _ => (),
        }
    }
    fn mouse_button_up_event(&mut self, _ctx: &mut Context, button: MouseButton, x: f32, y: f32){
        match button {
            MouseButton::Left => {
                if let Some(ball)= &self.ghost{
                    let xdir : f32 = (x-ball.x)/10.0;
                    let ydir : f32 = (y-ball.y)/10.0;
                    let mut overlap = false;
                    for ball2 in &self.balls {
                        let dx = ball.x-ball2.x;
                        let dy = ball.y-ball2.y;
                        if dx*dx+dy*dy<(20.0+20.0)*(20.0+20.0) {
                            overlap = true;
                        }
                    }
                    if overlap {
                        println!("There's already something there.");
                    }else{
                        self.balls.push(Ball{x:ball.x,y:ball.y,xdir:xdir,ydir:ydir});
                    }
                }
                self.ghost = None;
            }
            _ => (),
        }
    }
}

