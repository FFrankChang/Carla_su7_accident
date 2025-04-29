import threading
import carla
import pygame
import weakref
import random
import time
from datetime import datetime
import numpy as np
import os
import math
from sensor_script.steering_angle import parse_euler, get_steering_angle
from sensor_script.pedal import get_data,pedal_receiver

system_fault = False
init_location = carla.Location(x=-400, y=-31.5, z=15)

def change_weather(world):
    weather = carla.WeatherParameters(
        cloudiness=100,
        precipitation=0.0,
        precipitation_deposits=0.0,
        wind_intensity=0.0,
        sun_azimuth_angle=30.0,
        sun_altitude_angle=90.0
    )
    world.set_weather(weather)

def get_sensor_data():
    K1 = 0.55
    steer = get_steering_angle() / 450
    steerCmd = K1 * math.tan(1.1 * steer)
    acc,brake = get_data()
    if acc > 0.1:
        brake =0
    # print(steerCmd, acc, brake)
    return  steerCmd, acc, brake 

class Vehicle_Traffic:
    def __init__(self, world, tm_port=8000):
        self.world = world
        self.env_map = self.world.get_map()
        self.blueprint_library = self.world.get_blueprint_library()
        self.tm = client.get_trafficmanager(tm_port)
        self.tm.set_synchronous_mode(True)
        self.tm.global_percentage_speed_difference(-302)

    def create_main_vehicle(self, points=None, vehicle_model=None):
        vehicles = []
        if vehicle_model:
            blueprint_car = self.blueprint_library.filter('*vehicle*')
            cars = [bp for bp in blueprint_car if vehicle_model in bp.id.lower()]
        else:
            cars = self.blueprint_library.filter('*tesla*')

        for index, point in enumerate(points):
            waypoint = self.env_map.get_waypoint(point)
            transform = carla.Transform(point, waypoint.transform.rotation)
            
            vehicle = self.world.try_spawn_actor(random.choice(cars), transform)
            if vehicle:
                vehicles.append(vehicle)
                light_front = carla.VehicleLightState.HighBeam
                vehicle.set_light_state(carla.VehicleLightState(light_front))
                
                vehicle.set_autopilot(True)  
            else:
                print(f"Vehicle {index} failed to spawn!")
        return vehicles

    def attach_collision_sensor(self, vehicle, callback):
        blueprint = self.blueprint_library.find('sensor.other.collision')
        collision_sensor = self.world.spawn_actor(blueprint, carla.Transform(), attach_to=vehicle)
        collision_sensor.listen(callback)
        return collision_sensor

class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude
        # print(f"Latitude: {self.lat}, Longitude: {self.lon}")  # Real-time latitude and longitude

class Main_Car_Control:
    def __init__(self, main_car, world, window, instantaneous_speed=False):
        self.vehicle = main_car
        self.instantaneous_speed = instantaneous_speed
        self.world = world
        self.autopilot_flag = False
        self.speed_limit = 100
        self.steer = 0
        self.throttle = 0
        self.brake = 0
        self.window = window
        self.collision_occurred = False
        self.start_time = time.time()
        self.collision_time = None
        self.running = True

    def follow_road(self):
        while self.running:
            self.speed = self.get_speed()
            self.window.speed = self.speed


    def stop_scenario(self):
        self.running = False
        self.world.wait_for_tick()
        for vehicle in self.world.get_actors().filter('vehicle.*'):
            vehicle.apply_control(carla.VehicleControl(hand_brake=True, throttle=0.0))
        print("Scenario paused due to collision.")

    def collision_event(self, event):
        if not self.collision_occurred:
            self.collision_occurred = True
            self.collision_time = time.time() - self.start_time
            collision_message = f"Collision! {self.collision_time:.2f} s"
            self.window.set_collision_info(collision_message)  # 设置窗口中显示的碰撞信息
            print(collision_message)
            self.stop_scenario()  

    def get_speed(self):
        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return int(speed * 3.6)

class Window:
    def __init__(self, world, blueprint_library, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1920, 360
        self.screen = None
        self.fonts = {}
        pygame.init()
        self.collision_info = None

        self.clock = pygame.time.Clock()
        self.size = 18
        self.fps = 90
        self.font = pygame.font.Font(r"TTF\宋体.ttf", self.size)

        self.blueprint_camera = blueprint_library.find('sensor.camera.rgb')
        self.blueprint_camera.set_attribute('image_size_x', f'{self.SCREEN_WIDTH}')
        self.blueprint_camera.set_attribute('image_size_y', f'{self.SCREEN_HEIGHT}')
        self.blueprint_camera.set_attribute('fov', '140')
        spawn_point = carla.Transform(carla.Location(x=1.8, y = -0.3, z=1.25), carla.Rotation(pitch=-8, yaw=0, roll=0))
        self.sensor = self.world.spawn_actor(self.blueprint_camera, spawn_point, attach_to=self.vehicle)
        self.show_esc = False
        self.start_show_esc_after = 10
        self.show_duration = 3
        self.start_time = time.time()
        self.show_esc_time = self.start_time + self.start_show_esc_after
        self.end_esc_time = self.show_esc_time + self.show_duration
        self.speed = 0

        threading.Thread(target=self.show_screen).start()

    def show_screen(self):
        pygame.display.set_caption("xiaomi_su7 accident")
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT), pygame.RESIZABLE, 32)
        self.sensor.listen(lambda image: self.process_img(image))
        while True:
            current_time = time.time()
            if self.show_esc_time <= current_time < self.end_esc_time:
                self.show_esc = True
                global system_fault
                system_fault = True
            else:
                self.show_esc = False

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    os._exit(0)
                elif event.type == pygame.VIDEORESIZE:
                    self.CreateWindow(event.w, event.h)
            pygame.display.update()
            self.clock.tick(self.fps)

    def process_img(self, image):
        i = np.array(image.raw_data)
        i2 = i.reshape((self.SCREEN_HEIGHT, self.SCREEN_WIDTH, 4))
        i3 = i2[:, :, :3]
        i3 = np.rot90(i3, k=1, axes=(0, 1))
        i3 = i3[..., ::-1]
        img_surface = pygame.surfarray.make_surface(np.flip(i3, axis=0))
        self.screen.blit(img_surface, (0, 0))

        if self.collision_info:
            self.draw_text(self.collision_info, 150, (self.SCREEN_WIDTH // 2 -200, self.SCREEN_HEIGHT // 3 -50), bold=True, color=(255, 255, 255))
        self.draw_text(f"{self.speed}", 50, (self.SCREEN_WIDTH // 2, self.SCREEN_HEIGHT // 1.8), bold=True,color=(255, 255, 255))

    def draw_text(self, word, size, position, bold=False, color=(255, 0, 0)):
        font_key = (size, bold)
        if font_key not in self.fonts:
            font = pygame.font.Font(r"TTF\宋体.ttf", size)
            font.set_bold(bold)
            self.fonts[font_key] = font
        else:
            font = self.fonts[font_key]
        
        text = font.render(word, True, color)
        text_rect = text.get_rect()
        text_rect.topleft = position
        self.screen.blit(text, text_rect)

    def set_collision_info(self, info):
        self.collision_info = info


def destroy_all_vehicles_traffics(world, vehicle_flag=True, traffic_flag=True):
    actors = []
    if vehicle_flag:
        actors += list(world.get_actors().filter('*vehicle*'))
    if traffic_flag:
        actors += list(world.get_actors().filter("*prop*"))
    for actor in actors:
        actor.destroy()

def scene_jian(main_car_control):  
    threading.Thread(target=main_car_control.follow_road).start()

if __name__ == '__main__':
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(60)
    world = client.get_world()
    env_map = world.get_map()
    spectator = world.get_spectator()
    blueprint_library = world.get_blueprint_library()
    prop_model = blueprint_library.filter('*prop*')
    change_weather(world)
    pygame.init()
    pygame.mixer.init()
    threading.Thread(target=pedal_receiver).start()
    threading.Thread(target=parse_euler,daemon=True).start()
    destroy_all_vehicles_traffics(world)
    vehicle_traffic = Vehicle_Traffic(world)
    vehicle = vehicle_traffic.create_main_vehicle([init_location], vehicle_model="vehicle.tesla.model3")[0]
    window = Window(world, vehicle_traffic.blueprint_library, vehicle)
    gnss_sensor = GnssSensor(vehicle)  
    main_car_control = Main_Car_Control(vehicle, world, window, True)
    collision_sensor = vehicle_traffic.attach_collision_sensor(vehicle, main_car_control.collision_event)
    scene_jian(main_car_control)

    while True:
        world.tick()
        time.sleep(0.01)
