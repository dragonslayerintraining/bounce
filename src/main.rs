use ggez::{graphics, Context, ContextBuilder, GameResult};
use ggez::event::{self,EventHandler};
use ggez::input::mouse::MouseButton;
use ggez::nalgebra::{Vector2, Point2};

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
    pos: Point2<f32>,
    dir: Vector2<f32>,
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

//Returns 2 roots in sorted order, or None if fewer
fn solve_quadratic(a: f32, b: f32, c: f32) -> Option<(f32,f32)>{
    if a == 0.0 {return None;}
    let discr = b*b-4.0*a*c;
    if discr <= 0.0 {return None;}
    //Numerically stable formulas
    if b >= 0.0 {
        let tmp = (-b-discr.sqrt())/2.0;
        return Some((tmp/a,c/tmp));
    }else{
        let tmp = (-b+discr.sqrt())/2.0;
        return Some((c/tmp,tmp/a));
    }
}

impl World{
    fn new(_ctx:&mut Context) -> World{
        /*
        //Useful for reproducibility
        let mut balls = vec![];
        for i in 0..5 {
            for j in 0..5 {
                let x = (i as f32)*80.0+50.0;
                let y = (j as f32)*80.0+50.0;
                balls.push(Ball{pos:Point2::new(x,y),dir:Vector2::new(0.0,0.0)});
            }
        }
        balls.push(Ball{pos:Point2::new(50.0,450.0),dir:Vector2::new(-10.0,50.0)});
        
        World{
            balls: balls,
            ghost: None,
        }
        */
        World{
            balls: vec![],
            ghost: None,
        }
    }
    fn simple_update(&mut self, dt: f32) {
        for ball in &mut self.balls {
            ball.pos+=ball.dir*dt;
        }
    }
    fn update(&mut self, step: f32) {
        let mut dt = step;
        loop {
            let mut til: f32 = dt;
            let mut ev: PhysicsEvent = PhysicsEvent::NoEvent;
            for (i, ball) in self.balls.iter().enumerate() {
                if ball.dir.x < 0.0 {
                    let t = (20.0-ball.pos.x)/ball.dir.x;
                    if til > t { til=t; ev = PhysicsEvent::BallHWallCollision(i); }
                }
                if ball.dir.x > 0.0 {
                    let t = (500.0-20.0-ball.pos.x)/ball.dir.x;
                    if til > t { til=t; ev = PhysicsEvent::BallHWallCollision(i); }
                }
                if ball.dir.y < 0.0 {
                    let t = (20.0-ball.pos.y)/ball.dir.y;
                    if til > t { til=t; ev = PhysicsEvent::BallVWallCollision(i); }
                }
                if ball.dir.y > 0.0 {
                    let t = (500.0-20.0-ball.pos.y)/ball.dir.y;
                    if til > t { til=t; ev = PhysicsEvent::BallVWallCollision(i); }
                }
            }
            for (i, ball1) in self.balls.iter().enumerate() {
                for (j, ball2) in self.balls.iter().enumerate() {
                    //use frame of reference of first ball
                    let relpos = ball2.pos - ball1.pos;
                    let reldir = ball2.dir - ball1.dir;
                    //solve ||reldir*t+relpos||=||R|| for t
                    //||reldir||^2*t^2+2*reldir*relpos*t+||relpos||^2=||R||^2
                    //Rewrite as the quadratic at^2+bt+c=0
                    let a = reldir.dot(&reldir);
                    let b = 2.0*reldir.dot(&relpos);
                    let c = relpos.dot(&relpos)-(20.0+20.0)*(20.0+20.0);
                    if let Some((t1,t2)) = solve_quadratic(a,b,c) {
                    assert!(t1<=t2);
                    //Balls should never actually overlap
                    //But rounding error can be a problem here
                    if t1 < -0.0 && t2 > 0.00001 { println!("Overlapping balls! t={}..{}",t1,t2); }
                    //If no overlap in the future, ignore
                    if t1 <= 0.0 {continue;}
                    //Register collision
                    if til > t1 { til=t1; ev = PhysicsEvent::BallBallCollision(i,j); }
                    }
                }
            }
            assert!(til>=0.0);
            self.simple_update(til);
            dt-=til;
            match ev {
                PhysicsEvent::NoEvent => break,
                PhysicsEvent::BallHWallCollision(i) => self.balls[i].dir.x *= -1.0,
                PhysicsEvent::BallVWallCollision(i) => self.balls[i].dir.y *= -1.0,
                PhysicsEvent::BallBallCollision(i,j) => {
                    assert!(i!=j);
                    //In the absence of rounding error, ||balls[i].pos-balls[j].pos|||=R1+R2
                    //But there is rounding error
                    let bouncedir = self.balls[j].pos-self.balls[i].pos;
                    let bouncedir = bouncedir/bouncedir.dot(&bouncedir).sqrt();
                    let reldir = self.balls[j].dir-self.balls[i].dir;
                    let impulse = bouncedir.dot(&reldir)*bouncedir;
                    self.balls[i].dir+=impulse;
                    self.balls[j].dir-=impulse;
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
            graphics::draw(ctx,&circle,(ball.pos,))?;
        }
        let ghost_circle = graphics::Mesh::new_circle(ctx,graphics::DrawMode::fill(),ggez::nalgebra::Point2::new(0.0,0.0),20.0,2.0,[0.0,0.0,0.0,0.5].into())?;
        if let Some(ball) = &self.ghost {
            graphics::draw(ctx,&ghost_circle,(ball.pos,))?;
        }
        graphics::present(ctx)?;
        Ok(())
    }
    fn mouse_button_down_event(&mut self, _ctx: &mut Context,button: MouseButton, x: f32, y: f32){
        match button {
            MouseButton::Left => self.ghost = Some(Ball{pos:Point2::new(x,y),dir:Vector2::new(0.0,0.0)}),
            _ => (),
        }
    }
    fn mouse_button_up_event(&mut self, _ctx: &mut Context, button: MouseButton, x: f32, y: f32){
        match button {
            MouseButton::Left => {
                if let Some(ball)= &self.ghost{
                    let xdir : f32 = (x-ball.pos.x)/10.0;
                    let ydir : f32 = (y-ball.pos.y)/10.0;
                    let mut overlap = false;
                    for ball2 in &self.balls {
                        let dr = ball.pos-ball2.pos;
                        if dr.dot(&dr)<(20.0+20.0)*(20.0+20.0) {
                            overlap = true;
                        }
                    }
                    if ball.pos.x < 20.0 || ball.pos.x > 500.0-20.0 ||
                        ball.pos.y < 20.0 || ball.pos.y > 500.0-20.0 {
                            overlap = true;
                        }
                    if overlap {
                        println!("There's already something there.");
                    }else{
                        self.balls.push(Ball{pos:ball.pos,dir:Vector2::new(xdir,ydir)});
                        println!("Added ball {}",self.balls.len());
                    }
                }
                self.ghost = None;
            }
            _ => (),
        }
    }
}

