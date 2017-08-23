import pygame as pg
import numpy as np
import random as rnd


class CosmicSprite(pg.sprite.Sprite):
    def __init__(self,
                 sprite_img,
                 mass=1.,
                 velocity=[0., 0.],
                 position=[0., 0.],
                 angular_velocity=0.,
                 angle=0.,
                 friction=.2,
                 ):
        pg.sprite.Sprite.__init__(self)
        self.image = sprite_img
        self.rect = self.image.get_rect()
        self.mass = mass
        self.velocity = np.array(velocity)
        self.angular_velocity = angular_velocity
        self.position = np.array(position)
        self.angle = angle
        self.radius = max(self.rect.width, self.rect.height)
        self.friction = friction
        self._position_before_update = self.position[:]
        self._angle_before_update = angle
        self._update_rect()

    def _update_rect(self):
        self.rect.left = self.position[0] - self.rect.width/2.
        self.rect.top = self.position[1] - self.rect.height/2.

    def on_collision(self, other):
        if isinstance(other, CosmicSprite):
            self.velocity *= (self.friction*other.friction)
            self.angular_velocity *= (self.friction*other.friction)
            self.position = self._position_before_update

    def update_gravity(self, dt=.04, gravity_objects=[]):
        for gravity_object in gravity_objects:
            object_position = gravity_object.position
            gravity_vector = object_position-self.position
            gravity_distance = np.linalg.norm(gravity_vector)
            gravity_acceleration = float('6.7e-11')*gravity_object.mass/(gravity_distance**2)
            self.velocity += gravity_acceleration*dt

    def update(self, dt=.04):
        pg.sprite.Sprite.update(self)
        self._position_before_update = self.position[:]
        self._angle_before_update = self.angle
        self.position = self.position + self.velocity*dt
        angle_increment = self.angular_velocity*dt
        self.angle = (self.angle + angle_increment) % 2*np.pi

        self.image = pg.transform.rotate(self.image, angle_increment)
        self._update_rect()

class CosmicSpriteDecorator(CosmicSprite):
    def __init__(self, cosmic_sprite):
        self.cosmic_sprite = cosmic_sprite

class Ship(CosmicSpriteDecorator):
    angular_thruster_cost = 1.
    position_thruster_cost = 1.
    crash_fuel_cost = 1.
    crash_threshold = 1.

    def __init__(self,
                 cosmic_sprite,
                 angular_thrusters=0.,
                 position_thrusters=0.,
                 fuel=100.
                 ):
        CosmicSpriteDecorator.__init__(self, cosmic_sprite)
        self.angular_thrusters = angular_thrusters
        self.position_thrusters = position_thrusters
        self.fuel = fuel

    def __getattr__(self, item):
        if item in self.__dict__.keys():
            return getattr(self, item)
        else:
            return getattr(self.cosmic_sprite, item)

    def on_collision(self, other):
        velocity_diff = np.linalg.norm(self.velocity - other.velocity)
        energy = self.mass*(velocity_diff**2)/2
        crash_damage = max(0., energy - Ship.crash_threshold)
        self.fuel -= crash_damage*Ship.crash_fuel_cost
        self.fuel = max(0., self.fuel)
        CosmicSprite.on_collision(self, other)


    def update_gravity(self, dt=.04, gravity_objects=[]):
        self.cosmic_sprite.update_gravity(dt, gravity_objects)

    def update(self, dt=.04):
        position_thrust = self.position_thrusters*dt
        self.fuel -= max(0., np.abs(position_thrust)*Ship.position_thruster_cost)

        angular_thrust = self.angular_thrusters*dt
        self.fuel -= max(0., np.abs(angular_thrust)*Ship.angular_thruster_cost)

        origin = self.cosmic_sprite.position
        direction = np.array([0., 1.])
        angle = self.cosmic_sprite.angle
        rotated_direction = np.array([direction[0]*np.cos(angle) - direction[1]*np.sin(angle),
                                      direction[1]*np.cos(angle) + direction[0]*np.sin(angle)]).reshape(2)

        position_velocity_inc = rotated_direction*position_thrust

        angular_velocity_inc = angular_thrust

        if self.fuel > 0.:
            self.cosmic_sprite.velocity += position_velocity_inc
            self.cosmic_sprite.angular_velocity += angular_velocity_inc

        self.cosmic_sprite.update(dt)

def main():
    pg.init()
    screen_size = (640, 320)
    screen = pg.display.set_mode(screen_size)

    images_dict = {
        'ship': pg.image.load(r'./img/ship.png'),
        'planet': pg.image.load(r'./img/planet.png')
    }

    ships = pg.sprite.Group()
    ships.add(Ship(CosmicSprite(sprite_img=images_dict['ship'],
                                position=[30., 30.],
                                angular_velocity=0.01)))
    planets = pg.sprite.Group()
    planets.add(CosmicSprite(sprite_img=images_dict['planet'],
                             position=[screen_size[0]/2,
                                       screen_size[1]/2],
                             angular_velocity=0.01),
                             )

    clock = pg.time.Clock()

    while True:
        screen.fill((0,0,0))
        ships.draw(screen)
        planets.draw(screen)

        #ships.update(0.5)
        planets.update(0.5)

        pg.display.update()

        clock.tick(30)



if __name__ == '__main__':
    main()
