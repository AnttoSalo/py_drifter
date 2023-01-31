import pygame
import pymunk
import pymunk.pygame_util
import math
from pygame import Vector2
import pymunk.constraints


class App:
    def __init__(self):
        pygame.init()
        self.screen_width = 900
        self.screen_height = 600
        self.window = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.draw_options = pymunk.pygame_util.DrawOptions(self.window)
        self.running = True
        self.space = pymunk.Space()
        self.space.gravity = 0, 0
        
        background = pymunk.Body(body_type=pymunk.Body.STATIC)
        background_shape = pymunk.Poly.create_box(background, (self.screen_width*2, self.screen_height*2))
        self.background_shapefilter = pymunk.ShapeFilter(group=1)
        background_shape.friction = 0.5        
        self.space.add(background, background_shape)
        
        self.fps = 60
        self.clock = pygame.time.Clock()
        self.dt = 1/self.fps
            
    def run(self):
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                    pygame.image.save(self.window, 'intro.png')
            car.steering()
            self.draw()   
            self.clock.tick(self.fps)
            self.space.step(0.01)
    def draw(self):
        self.window.fill((255,255,255))
        self.space.debug_draw(self.draw_options)
        car.draw()
        pygame.display.update()
app = App()

class Car:
    def __init__(self):
        self.mass = 1000
        self.pos = (400,300)
        self.moment = 100000
        self.body = pymunk.Body(self.mass, self.moment, body_type=pymunk.Body.DYNAMIC)
        self.body.width = 50
        self.body.length = 100
        self.max_steering_angle = 15
        self.steering_angle = 0
        self.body_shape = pymunk.Poly.create_box(self.body, (self.body.width, self.body.length))
        app.space.add(self.body, self.body_shape)
        self.car_speed = 10

        self.wheel_width = 10
        self.wheel_height = self.wheel_width * 2
        self.wheelbase_length = self.body.length - 50
        self.body.position = (self.pos)
        
        #alternattive approach
        self.carLocation = Vector2(400,300)
        """self.frontWheel = Vector2(self.carLocation + self.wheelBase/2 * Vector2( math.cos(self.carHeading) , math.sin(self.carHeading) ))
        self.backWheel = Vector2(self.carLocation - self.wheelBase/2 * Vector2( math.cos(self.carHeading) , math.sin(self.carHeading) ))
        self.steering_angle = 0  """
        self.wheel_mass = 1000
        self.wheel_moment = 100

        self.front_left = pymunk.Body(self.wheel_mass, self.wheel_moment, pymunk.Body.DYNAMIC)
        self.front_left.position = (self.pos[0]-self.body.width/2+self.wheel_width/2, self.pos[1] - 25)
        self.front_left_shape = pymunk.Poly.create_box(self.front_left, (self.wheel_width, self.wheel_height))
        self.front_left_shape.friction = 0
        self.front_left_shape.color = (0, 0, 0, 1)
        app.space.add(self.front_left, self.front_left_shape)

        self.front_right = pymunk.Body(self.wheel_mass, self.wheel_moment, pymunk.Body.DYNAMIC)
        self.front_right.position = (self.pos[0] + self.body.width/2-self.wheel_width/2, self.pos[1] - 25)
        self.front_right_shape = pymunk.Poly.create_box(self.front_right, (self.wheel_width, self.wheel_height))
        self.front_right_shape.friction = 0
        self.front_right_shape.color = (0, 0, 0, 1)
        app.space.add(self.front_right, self.front_right_shape)

        self.rear_left = pymunk.Body(self.wheel_mass, self.wheel_moment, pymunk.Body.DYNAMIC)
        self.rear_left.position = (self.pos[0] -self.body.width/2+self.wheel_width/2, self.pos[1] + 25)
        self.rear_left_shape = pymunk.Poly.create_box(self.rear_left, (self.wheel_width, self.wheel_height))
        self.rear_left_shape.friction = 0
        self.rear_left_shape.color = (0, 0, 0, 1)
        app.space.add(self.rear_left, self.rear_left_shape)

        self.rear_right = pymunk.Body(self.wheel_mass, self.wheel_moment, pymunk.Body.DYNAMIC)
        self.rear_right.position = (self.pos[0] + self.body.width/2-self.wheel_width/2, self.pos[1] + 25)
        self.rear_right_shape = pymunk.Poly.create_box(self.rear_right, (self.wheel_width, self.wheel_height))
        self.rear_right_shape.friction = 0
        self.rear_right_shape.color = (0, 0, 0, 1)
        app.space.add(self.rear_right, self.rear_right_shape)

        # Set the collision type for the body
        self.body_shape.filter = pymunk.ShapeFilter(group=1)

        # Set the collision type for the wheels
        self.front_left_shape.filter = pymunk.ShapeFilter(group=1)
        self.front_right_shape.filter= pymunk.ShapeFilter(group=1)
        self.rear_left_shape.filter = pymunk.ShapeFilter(group=1)
        self.rear_right_shape.filter = pymunk.ShapeFilter(group=1)
        
        self.wheel_constaint_max_force = math.inf        
        self.front_left_constraint = pymunk.constraints.PivotJoint(self.front_left,self.body,(0,0),(-self.body.width/2+self.wheel_width/2,-25))
        self.front_left_constraint.max_force = self.wheel_constaint_max_force
        app.space.add(self.front_left_constraint)
        
        self.front_right_constraint = pymunk.constraints.PivotJoint(self.front_right,self.body,(0,0),(self.body.width/2-self.wheel_width/2,-25))
        self.front_right_constraint.max_force = self.wheel_constaint_max_force
        app.space.add(self.front_right_constraint)

        self.rear_left_constraint = pymunk.constraints.PivotJoint(self.rear_left,self.body,(0,0),(-self.body.width/2+self.wheel_width/2,25))
        self.rear_left_constraint.max_force = self.wheel_constaint_max_force
        app.space.add(self.rear_left_constraint)

        self.rear_right_constraint = pymunk.constraints.PivotJoint(self.rear_right,self.body,(0,0),(self.body.width/2-self.wheel_width/2,25))
        self.rear_right_constraint.max_force = self.wheel_constaint_max_force
        app.space.add(self.rear_right_constraint)
        
    
    def draw(self):
        #alternattive approach
        """self.carHeading = math.atan2( self.frontWheel.Y - self.backWheel.Y , self.frontWheel.X - self.backWheel.X )
        self.backWheel += self.car_speed *  Vector2(math.cos(self.carHeading) , math.sin(self.carHeading))
        self.frontWheel += self.car_speed * Vector2(math.cos(self.carHeading+self.steerAngle) , math.sin(self.carHeading+self.steerAngle)) """
        
    def steering(self):
        pressed = pygame.key.get_pressed()
        if pressed[pygame.K_a]:
            self.steering_angle = 1 * self.max_steering_angle
        elif pressed[pygame.K_d]:
            self.steering_angle = -1 * self.max_steering_angle
        else: 
            self.steering_angle = 0
        self.front_left.angle = self.steering_angle
        self.front_right.angle = self.steering_angle
        if pressed[pygame.K_w]:
            #self.body.apply_force_at_local_point((100,-1000),(0,0))
            self.rear_left.apply_force_at_local_point((0,-10000),(0,0))
            self.rear_right.apply_force_at_local_point((0,-10000),(0,0))
        
car = Car()
app.run()