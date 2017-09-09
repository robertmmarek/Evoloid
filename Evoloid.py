import pygame as pg
import numpy as np
import copy as copy
import random as rnd


class View(pg.sprite.Sprite):
    def __init__(self, cosmic_object):
        pg.sprite.Sprite.__init__(self)
        self.cosmic_object = cosmic_object

    def update_sprite(self, scale=1.):
        pass

    def draw(self, screen, offset=[0., 0.]):
        pass


class AutoScalingWorldView(View):
    def __init__(self, sub_views, max_scale=1.5, min_scale=0.1, output_size=(640., 480.)):
        self.buffer_screen = pg.Surface(output_size)
        self.sub_views = sub_views
        self.max_scale = max_scale
        self.min_scale = min_scale
        self.output_size = output_size
        self._mean_pos = np.array([0., 0.])

    def update_sprite(self, scale=1.):
        self.buffer_screen.fill((0., 0., 0.))
        mean_pos = np.array([0., 0.])
        max_x = max([view.cosmic_object.position[0] for view in self.sub_views])
        min_x = min([view.cosmic_object.position[0] for view in self.sub_views])
        max_y = max([view.cosmic_object.position[1] for view in self.sub_views])
        min_y = min([view.cosmic_object.position[1] for view in self.sub_views])

        x_range = (max_x-min_x)*2
        y_range = (max_y-min_y)*2
        scale_value = np.linalg.norm([x_range, y_range])

        autoscale = 0.9*min(self.output_size[0]/scale_value, self.output_size[1]/scale_value)
        autoscale = scale*max(self.min_scale, min(self.max_scale, autoscale))

        for view in self.sub_views:
            mean_pos += view.cosmic_object.position*view.cosmic_object.mass
            view.update_sprite(autoscale)

        mean_pos /= float(sum([view.cosmic_object.mass for view in self.sub_views]))
        self._mean_pos = mean_pos

        draw_offset = np.array(self.output_size)/2. - mean_pos*autoscale

        for view in self.sub_views:
            view.draw(self.buffer_screen, draw_offset)

    def draw(self, screen, offset=[0., 0.]):
        View.draw(self, screen)
        screen.blit(self.buffer_screen, offset)


class CosmicObjectImageView(View):
    def __init__(self, cosmic_object, image):
        View.__init__(self, cosmic_object)
        self._base_image = image
        self.image = image
        self.rect = self.image.get_rect()

    def update_sprite(self, scale=1.):
        View.update_sprite(scale)
        angle_in_deg = np.rad2deg(self.cosmic_object.angle + np.pi/2.)
        previous_center = (self.cosmic_object.position[0]*scale, self.cosmic_object.position[1]*scale)
        self.image = pg.transform.rotate(self._base_image, -angle_in_deg)
        image_rect = self.image.get_rect()
        base_image_rect = self._base_image.get_rect()
        max_dimension = max(base_image_rect.width, base_image_rect.height)
        resize_factor = 2*self.cosmic_object.radius*scale/max_dimension
        self.image = pg.transform.scale(self.image, (int(image_rect.width*resize_factor),
                                                     int(image_rect.height*resize_factor)))

        self.rect = self.image.get_rect()
        self.rect.center = previous_center

    def draw(self, screen, offset=[0., 0.]):
        moved_rect = self.rect.copy()
        moved_rect = moved_rect.move(offset[0], offset[1])
        screen.blit(self.image, moved_rect)


class ShipImageView(CosmicObjectImageView):
    def __init__(self,
                 ship,
                 ship_image,
                 position_thrust_image,
                 angular_thrust_image_positive,
                 angular_thrust_image_negative):
        CosmicObjectImageView.__init__(self, ship, ship_image)
        self.ship = ship
        self.position_thrust_image = position_thrust_image
        self.angular_thrust_image_negative = angular_thrust_image_negative
        self.angular_thrust_image_positive = angular_thrust_image_positive

        self.position_thrust_view = None
        self.angular_thrust_view = None

    def update_sprite(self, scale=1.):
        CosmicObjectImageView.update_sprite(self, scale)

        if self.ship.fuel > 0. and np.abs(self.ship.angular_thrusters) > 0.:
            angular_thrust_image = self.angular_thrust_image_positive
            if self.ship.angular_thrusters < 0.:
                angular_thrust_image = self.angular_thrust_image_negative
            angular_thrust_object = copy.deepcopy(self.ship.cosmic_object)
            self.angular_thrust_view = CosmicObjectImageView(angular_thrust_object, angular_thrust_image)
            self.angular_thrust_view.update_sprite(scale)
        else:
            self.angular_thrust_view = None

        if self.ship.fuel > 0. and np.abs(self.ship.position_thrusters) > 0.:
            position_thrust_object = copy.deepcopy(self.ship.cosmic_object)
            position_thrust_object.position += -.75*self.ship.radius*self.ship.direction
            self.position_thrust_view = CosmicObjectImageView(position_thrust_object, self.position_thrust_image)
            self.position_thrust_view.update_sprite(scale)
        else:
            self.position_thrust_view = None

    def draw(self, screen, offset=[0., 0.]):
        if self.angular_thrust_view is not None:
            self.angular_thrust_view.draw(screen, offset)

        if self.position_thrust_view is not None:
            self.position_thrust_view.draw(screen, offset)

        CosmicObjectImageView.draw(self, screen, offset)


class CosmicObject(pg.sprite.Sprite):
    @staticmethod
    def collision(cosmic_object_a, cosmic_object_b):
        ret = pg.sprite.collide_circle(cosmic_object_a, cosmic_object_b)
        if ret:
            cosmic_object_a.on_collision(cosmic_object_b)
            cosmic_object_b.on_collision(cosmic_object_a)

        return ret

    @staticmethod
    def normalize_array(array):
        norm = np.linalg.norm(array)
        if norm != 0.:
            array /= norm

        return array

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

        self.direction = self._calc_direction_vector()

        self._update_rect()

    def _calc_direction_vector(self):
        return np.array([np.cos(self.angle), np.sin(self.angle)])

    def _fix_collision(self, other):
        collision_vector = other.position-self.position
        collision_len = max(0., (self.radius+other.radius)-np.linalg.norm(collision_vector))
        if not self.fixed:
            self.position = self.position - CosmicObject.normalize_array(collision_vector)*collision_len

    def on_collision(self, other):
        if isinstance(other, CosmicObject):
            if not self.fixed:
                self.velocity = (self.velocity*self.mass+other.velocity*other.mass)/(self.mass+other.mass)
                self.angular_velocity = (self.angular_velocity*self.mass +
                                         other.angular_velocity*other.mass)/(self.mass+other.mass)
            self._fix_collision(other)

    def _update_gravity(self, dt=.04, gravity_objects=[]):
        for gravity_object in gravity_objects:
            object_position = gravity_object.position
            gravity_vector = object_position-self.position
            gravity_distance = np.linalg.norm(gravity_vector)
            normalized_gravity_vector = CosmicObject.normalize_array(gravity_vector)

            if gravity_distance != 0.:
                gravity_acceleration = float('6.7e-3')*gravity_object.mass/(gravity_distance**2)
            else:
                gravity_acceleration = 0.

            if not self.fixed:
                self.velocity += normalized_gravity_vector*gravity_acceleration*dt

    def _update_rect(self):
        self.rect.center = (self.position[0], self.position[1])

    def update(self, dt=.04, gravity_objects=[]):
        pg.sprite.Sprite.update(self)
        self._position_before_update = self.position[:]
        self._angle_before_update = self.angle

        self._update_gravity(dt, gravity_objects)

        if not self.fixed:
            self.position = self.position + self.velocity*dt

        angle_increment = self.angular_velocity*dt
        self.angle = (self.angle + angle_increment) % (2*np.pi)
        self._update_rect()
        self.direction = self._calc_direction_vector()


class CosmicObjectDecorator(CosmicObject):
    def __init__(self, cosmic_object):
        self.cosmic_object = cosmic_object


class Ship(CosmicObjectDecorator):
    angular_thruster_cost = 50.
    position_thruster_cost = 400.
    crash_fuel_cost = 1.
    crash_threshold = 1.
    velocity_boost_fuel_cost = 1.
    steal_fuel_multiplier = 1.3
    bullet_timeout_multiplier = 2.
    bullet_radius = 2.

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

    def __setattr__(self, key, value):
        if key == 'fuel':
            self.__dict__[key] = value
            self.__dict__[key] = max(0., self.__dict__[key])
        else:
            super(CosmicObjectDecorator, self).__setattr__(key, value)

    def on_collision(self, other):
        velocity_diff = np.linalg.norm(self.velocity - other.velocity)
        energy = self.mass*(velocity_diff**2)/2.
        crash_damage = max(0., energy - Ship.crash_threshold)
        self.fuel -= crash_damage*Ship.crash_fuel_cost
        self.fuel = max(0., self.fuel)
        self.cosmic_object.on_collision(other)

    def update(self, dt=.04, gravity_objects=[]):
        position_thrust = self.position_thrusters*dt
        self.fuel -= max(0., np.abs(position_thrust)*Ship.position_thruster_cost)

        angular_thrust = self.angular_thrusters*dt
        self.fuel -= max(0., np.abs(angular_thrust)*Ship.angular_thruster_cost)

        direction = np.array([0., 1.])
        angle = self.cosmic_object.angle

        position_velocity_inc = self.direction*position_thrust

        angular_velocity_inc = angular_thrust

        if self.fuel > 0.:
            self.cosmic_object.velocity += position_velocity_inc
            self.cosmic_object.angular_velocity += angular_velocity_inc

        self.cosmic_object.update(dt, gravity_objects)

        def shoot(self, fuel_to_use):
            bullet_space_object = copy.deepcopy(self.cosmic_object)
            bullet_space_object.radius = Ship.bullet_radius
            bullet_space_object.position += self.direction*1.1*(self.radius+Ship.bullet_radius)
            bullet_space_object.velocity += bullet_space_object.direction*(fuel_to_use/Ship.velocity_boost_fuel_cost)
            bullet = FuelStealerBullet(bullet_space_object,
                                       self,
                                       fuel_to_use*Ship.steal_fuel_multiplier,
                                       fuel_to_use*Ship.bullet_timeout_multiplier)
            return bullet


class FuelStealerBullet(CosmicObjectDecorator):
    def __init__(self, cosmic_object, origin_ship, fuel_to_steal, timeout=1.):
        CosmicObjectDecorator(self, cosmic_object)
        self.origin_ship = origin_ship
        self.fuel_to_steal = fuel_to_steal
        self.timeout = timeout
        self.deadly = True

    def on_collision(self, other):
        if isinstance(other, Ship) and self.deadly:
            previous_fuel = other.fuel
            other.fuel -= self.fuel_to_steal
            self.origin_ship.fuel += (previous_fuel-other.fuel)
            self.deadly = False
        self.cosmic_object.on_collision(other)

    def update(self, dt=0.04, gravity_objects=[]):
        self.timeout -= dt
        if self.timeout <= 0.:
            self.deadly = False
            self.timeout = 0.
        self.cosmic_object.update(dt, gravity_objects)


def main():
    pg.init()
    screen_size = (640, 640)
    screen = pg.display.set_mode(screen_size)

    images_dict = {
        'ship': pg.image.load(r'./img/ship.png'),
        'planet': pg.image.load(r'./img/planet.png'),
        'position_thrust': pg.image.load(r'./img/position_thrust.png'),
        'angular_negative_thrust': pg.image.load(r'./img/angular_negative_thrust.png'),
        'angular_positive_thrust': pg.image.load(r'./img/angular_positive_thrust.png')
    }

    ships = [Ship(CosmicObject(position=[320., 50.],
                               radius=10.,
                               angular_velocity=0.00,
                               velocity=[0., 0.],
                               mass=1.,
                               angle=-.75),
                               position_thrusters=0.0001,
                               angular_thrusters=0.000003
                               )
             ]
    planets = [CosmicObject(position=[320., 320.],
                            radius=30.,
                            mass=500.,
                            angular_velocity=0.,
                            fixed=True)]

    ships_view = [ShipImageView(ship,
                                images_dict['ship'],
                                images_dict['position_thrust'],
                                images_dict['angular_negative_thrust'],
                                images_dict['angular_positive_thrust']) for ship in ships]
    planets_view = [CosmicObjectImageView(planet, images_dict['planet']) for planet in planets]

    world_view = AutoScalingWorldView(ships_view+planets_view,
                                      max_scale=1.5,
                                      min_scale=.1,
                                      output_size=screen_size)

    global_clock = pg.time.Clock()

    while True:
        screen.fill((0, 0, 0))
        dt = global_clock.tick(60)

        for ship in ships:
            ship.update(dt, gravity_objects=planets+ships)

        for planet in planets:
            planet.update(dt, gravity_objects=planets+ships)

        collide_group = ships+planets
        collide_group_len = len(collide_group)
        for i in range(0, collide_group_len):
            element_a = collide_group.pop(0)
            for element_b in collide_group:
                CosmicObject.collision(element_a, element_b)

        world_view.update_sprite(1.)
        world_view.draw(screen)

        pg.display.update()


if __name__ == '__main__':
    main()
