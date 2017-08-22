import pygame as pg
import numpy as np


class CosmicSprite(pg.sprite.Sprite):
    def __init__(self,
                 sprite_img,
                 mass=1.,
                 velocity=[0., 0.],
                 position=[0., 0.],
                 angular_velocity=[0.],
                 angle=[0.],
                 ):
        pg.sprite.Sprite.__init__(self)
        self.image = sprite_img
        self.rect = self.image.get_rect()
        self.mass = mass
        self.velocity = np.array(velocity)
        self.angular_velocity = np.array(angular_velocity)
        self.position = np.array(position)
        self.angle = np.array(angle)
        self.radius = max(self.rect)

    def update_gravity(self, dt=.04, gravity_objects=[]):
        for gravity_object in gravity_objects:
            object_position = gravity_object.position
            gravity_vector = object_position-self.position
            gravity_distance = np.linalg.norm(gravity_vector)
            gravity_acceleration = float('6.7e-11')*gravity_object.mass/(gravity_distance**2)
            self.velocity += gravity_acceleration*dt

    def update(self, dt=.04):
        self.position = self.position + self.velocity*dt
        self.angle = (self.angle + self.angular_velocity*dt) % 2*np.pi

class CosmicSpriteDecorator(CosmicSprite):
    def __init__(self, cosmic_sprite):
        self.cosmic_sprite = cosmic_sprite

class Ship(CosmicSpriteDecorator):
    angular_thruster_cost = 1.
    position_thruster_cost = 1.

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

    def update_gravity(self, dt=.04, gravity_objects=[]):
        self.cosmic_sprite.update_gravity(dt, gravity_objects)

    def update(self, dt=.04):
        position_thrust = self.position_thrusters*dt
        self.fuel -= max(0., np.abs(position_thrust)*Ship.position_thruster_cost)

        angular_thrust = self.angular_thrusters*dt
        self.fuel -= max(0., np.abs(angular_thrust)*Ship.angular_thruster_cost)

        position_velocity_inc = np.array([0., 0.])
        origin = self.cosmic_sprite.position
        direction = np.array([0., 1.])
        angle = self.cosmic_sprite.angle
        rotated_direction = np.array([direction[0]*np.cos(angle) - direction[1]*np.sin(angle),
                                      direction[1]*np.cos(angle) + direction[0]*np.sin(angle)])
        position_velocity_inc = rotated_direction*position_thrust

        angular_velocity_inc = np.array([angular_thrust])

        if self.fuel > 0.:
            self.cosmic_sprite.velocity += position_velocity_inc
            self.cosmic_sprite.angular_velocity += angular_velocity_inc

        self.cosmic_sprite.update(dt)
