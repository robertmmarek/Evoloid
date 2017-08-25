import pygame as pg
import numpy as np
import random as rnd


class View(pg.sprite.Sprite):

    def __init__(self):
        pg.sprite.Sprite.__init__(self)

    def update_sprite(self, scale=1.):
        pass

    def draw(self, screen):
        pass


class CosmicObjectImageView(View):
    def __init__(self, cosmic_object, image):
        View.__init__(self)
        self.cosmic_object = cosmic_object
        self._base_image = image
        self.image = image
        self.rect = self.image.get_rect()

    def update_sprite(self, scale=1.):
        View.update_sprite(scale)

        angle_in_deg = np.rad2deg(self.cosmic_object.angle)
        previous_center = (self.cosmic_object.position[0]*scale, self.cosmic_object.position[1]*scale)
        self.image = pg.transform.rotate(self._base_image, angle_in_deg)
        image_rect = self.image.get_rect()
        base_image_rect = self._base_image.get_rect()
        max_dimension = max(base_image_rect.width, base_image_rect.height)
        resize_factor = 2*self.cosmic_object.radius*scale/max_dimension
        self.image = pg.transform.scale(self.image, (int(image_rect.width*resize_factor),
                                                     int(image_rect.height*resize_factor)))

        self.rect = self.image.get_rect()
        self.rect.center = previous_center

    def draw(self, screen):
        screen.blit(self.image, self.rect)


class CosmicObject(pg.sprite.Sprite):
    @staticmethod
    def collision(cosmic_object_a, cosmic_object_b):
        ret = pg.sprite.collide_circle(cosmic_object_a, cosmic_object_b)
        if ret:
            cosmic_object_a.on_collision(cosmic_object_b)
            cosmic_object_b.on_collision(cosmic_object_a)

        return ret

    @staticmethod
    def calculate_collision_velocity_scalar(cosmic_object_a, cosmic_object_b):
        diff_vector = CosmicObject.normalize_array(cosmic_object_a.position-cosmic_object_b.position)
        velocity_magnitude = np.linalg.norm(cosmic_object_a.velocity)
        if velocity_magnitude != 0.:
            normalized_velocity = cosmic_object_a.velocity/velocity_magnitude
        else:
            normalized_velocity = cosmic_object_a.velocity
        collision_velocity_scalar = np.sum(cosmic_object_a.velocity*diff_vector)
        return collision_velocity_scalar

    @staticmethod
    def normalize_array(array):
        norm = np.linalg.norm(array)
        if norm == 0.:
            norm = 1.

        return array/norm

    def __init__(self,
                 mass=1.,
                 velocity=[0., 0.],
                 position=[0., 0.],
                 angular_velocity=0.,
                 angle=0.,
                 friction=.2,
                 radius=1.,
                 fixed=False
                 ):
        pg.sprite.Sprite.__init__(self)
        self.fixed = fixed
        self.mass = mass
        self.velocity = np.array(velocity)
        self._update_velocity = self.velocity
        self.angular_velocity = angular_velocity
        self.position = np.array(position)
        self._update_position = self.position
        self.rect = pg.Rect(0, 0, 1., 1.)

        self.angle = angle
        self.radius = radius
        self.friction = friction
        self._position_before_update = self.position[:]
        self._angle_before_update = angle

        self._update_rect()

    def _fix_collision(self, other):
        diff_vector = (other.position-self.position)
        radius_sum = other.radius + self.radius
        fix_magnitude = max(0, radius_sum - np.linalg.norm(diff_vector))

        own_velocity_scalar = CosmicObject.calculate_collision_velocity_scalar(self, other)
        other_velocity_scalar = CosmicObject.calculate_collision_velocity_scalar(other, self)

        velocity_scalar_sum = (own_velocity_scalar+other_velocity_scalar)
        if velocity_scalar_sum != 0.:
            fix_factor = own_velocity_scalar/velocity_scalar_sum
        else:
            fix_factor = 0.

        if fix_factor == 0.:
            fix_factor = .5
        else:
            fix_factor = max(0., fix_factor)

        diff_direction = CosmicObject.normalize_array(diff_vector)

        if not self.fixed:
            self._update_position -= fix_magnitude*diff_direction*fix_factor
        else:
            self._update_position = self._position_before_update

    def on_collision(self, other):
        if isinstance(other, CosmicObject):
            if not self.fixed:
                self._update_velocity = (self.velocity*self.mass+other.velocity*other.mass)/(self.mass+other.mass)
            self.angular_velocity *= .75
            self._fix_collision(other)

    def update_gravity(self, dt=.04, gravity_objects=[]):
        for gravity_object in gravity_objects:
            object_position = gravity_object.position
            gravity_vector = object_position-self.position
            gravity_distance = np.linalg.norm(gravity_vector)
            normalized_gravity_vector = gravity_vector/gravity_distance
            gravity_acceleration = float('6.7e-3')*gravity_object.mass/(gravity_distance**2)
            self._update_velocity += normalized_gravity_vector*gravity_acceleration*dt

    def _update_rect(self):
        self.rect.center = (self.position[0], self.position[1])

    def update(self, dt=.04):
        pg.sprite.Sprite.update(self)
        self._position_before_update = self.position[:]
        self._angle_before_update = self.angle
        self.position = self._update_position
        self.velocity = self._update_velocity

        if not self.fixed:
            self.position = self.position + self.velocity*dt

        angle_increment = self.angular_velocity*dt
        self.angle = (self.angle + angle_increment) % (2*np.pi)
        self._update_rect()

        self._update_position = self.position
        self._update_velocity = self.velocity

    def resolve_collisions(self):
        self.position = self._update_position
        self.velocity = self._update_velocity


class CosmicObjectDecorator(CosmicObject):
    def __init__(self, cosmic_object):
        self.cosmic_object = cosmic_object


class Ship(CosmicObjectDecorator):
    angular_thruster_cost = 1.
    position_thruster_cost = 1.
    crash_fuel_cost = 1.
    crash_threshold = 1.

    def __init__(self,
                 cosmic_object,
                 angular_thrusters=0.,
                 position_thrusters=0.,
                 fuel=100.
                 ):
        CosmicObjectDecorator.__init__(self, cosmic_object)
        self.angular_thrusters = angular_thrusters
        self.position_thrusters = position_thrusters
        self.fuel = fuel

    def __getattr__(self, item):
        if item in self.__dict__.keys():
            return getattr(self, item)
        else:
            return getattr(self.cosmic_object, item)

    def on_collision(self, other):

        #if isinstance(other, Ship):
         #   print('hej')

        velocity_diff = np.linalg.norm(self.velocity - other.velocity)
        energy = self.mass*(velocity_diff**2)/2.
        crash_damage = max(0., energy - Ship.crash_threshold)
        self.fuel -= crash_damage*Ship.crash_fuel_cost
        self.fuel = max(0., self.fuel)
        self.cosmic_object.on_collision(other)

    def update_gravity(self, dt=.04, gravity_objects=[]):
        self.cosmic_object.update_gravity(dt, gravity_objects)

    def update(self, dt=.04):
        position_thrust = self.position_thrusters*dt
        self.fuel -= max(0., np.abs(position_thrust)*Ship.position_thruster_cost)

        angular_thrust = self.angular_thrusters*dt
        self.fuel -= max(0., np.abs(angular_thrust)*Ship.angular_thruster_cost)

        origin = self.cosmic_object.position
        direction = np.array([0., 1.])
        angle = self.cosmic_object.angle
        rotated_direction = np.array([direction[0]*np.cos(angle) - direction[1]*np.sin(angle),
                                      direction[1]*np.cos(angle) + direction[0]*np.sin(angle)]).reshape(2)

        position_velocity_inc = rotated_direction*position_thrust

        angular_velocity_inc = angular_thrust

        if self.fuel > 0.:
            self.cosmic_object.velocity += position_velocity_inc
            self.cosmic_object.angular_velocity += angular_velocity_inc

        self.cosmic_object.update(dt)


def main():
    pg.init()
    screen_size = (640, 640)
    screen = pg.display.set_mode(screen_size)

    images_dict = {
        'ship': pg.image.load(r'./img/ship.png'),
        'planet': pg.image.load(r'./img/planet.png')
    }

    ships = [Ship(CosmicObject(position=[320., 100.],
                               radius=10.,
                               angular_velocity=0.005,
                               velocity=[0., 0.],
                               mass=.1)),
             Ship(CosmicObject(position=[320., 50.],
                               radius=10.,
                               angular_velocity=0.005,
                               velocity=[0., 0.],
                               mass=.1))
             ]
    planets = [CosmicObject(position=[320., 320.],
                            radius=20.,
                            mass=500.,
                            angular_velocity=0.,
                            fixed=True)]

    ships_view = [CosmicObjectImageView(ship, images_dict['ship']) for ship in ships]
    planets_view = [CosmicObjectImageView(planet, images_dict['planet']) for planet in planets]

    global_clock = pg.time.Clock()

    while True:
        screen.fill((0, 0, 0))
        dt = global_clock.tick(30)

        for ship in ships:
            ship.update_gravity(dt, planets)
            ship.update(dt)

        for planet in planets:
            planet.update(dt)

        collide_group = ships+planets
        collide_group_len = len(collide_group)
        for i in range(0, collide_group_len):
            element_a = collide_group.pop(0)
            for element_b in collide_group:
                CosmicObject.collision(element_a, element_b)
                
        for object in ships+planets:
            object.resolve_collisions()

        for view in ships_view+planets_view:
            view.update_sprite(scale=1.)
            view.draw(screen)

        pg.display.update()


if __name__ == '__main__':
    main()
