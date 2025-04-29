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
import json
import redis
import paho.mqtt.client as mqtt

# https://ivwvideo.gambition.cn/hmi/xm-accident/audio/xiaomi.MP3
# https://ivwvideo.gambition.cn/hmi/xm-accident/audio/3.MP3
# https://ivwvideo.gambition.cn/hmi/xm-accident/audio/2.MP3
# https://ivwvideo.gambition.cn/hmi/xm-accident/audio/1.MP3
# https://ivwvideo.gambition.cn/hmi/xm-accident/audio/0.MP3
# https://ivwvideo.gambition.cn/hmi/xm-accident/img/1.jpg
# https://ivwvideo.gambition.cn/hmi/xm-accident/img/0.jpg

from sensor.steering_angle import parse_euler, get_steering_angle
from sensor.pedal import get_data,pedal_receiver

init_location = carla.Location(x=-400, y=-31.5, z=15)
background_vehicle_location = carla.Location(x=-300, y=-15, z=15)
redis_client = redis.StrictRedis(host='202.120.189.56', port=5010, password='Wanji@300552!', db=0)
mqtt_broker = "47.114.34.96"
mqtt_port = 1983
mqtt_topic = "HmiCommandTopic"
mqtt_topic = "YHAutoMatchResultData"
mqtt_client = mqtt.Client()
mqtt_client.connect(mqtt_broker, mqtt_port)
speed = {
    "speed":100
}
mqtt_msg = {
        "id": 0,
        "scence": 0,
        "type": 0,
        "hmiMsg": "前方障碍物请接管",
        "hmiImg": "https://ivwvideo.gambition.cn/hmi/xm-accident/img/0.jpg",
        "audio": "https://ivwvideo.gambition.cn/hmi/xm-accident/audio/xiaomi.MP3"
    }

def send_mqtt_message():
    mqtt_client.publish(mqtt_topic, json.dumps(mqtt_msg)) 

def send_to_redis(vehicleDictInfo):
    message = {
        "timestampType": "CREATE_TIME",
        "value": vehicleDictInfo,
        "timestamp": int(time.time() * 1000),
        "globalTimeStamp": int(time.time() * 1000),
        "pointList": []
    }
    redis_client.publish("sr_xm_data_channel", json.dumps(message))
    mqtt_client.publish("YHAutoMatchResultData",json.dumps(speed))
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
    # return 0,0,0
    K1 = 0.55
    steer = get_steering_angle() / 450
    steerCmd = K1 * math.tan(1.1 * steer)
    acc,brake = get_data()
    if acc > 0.1:
        brake = 0
    return steerCmd, acc, brake 

def car_control(vehicle, steer=0, throttle=1, brake=0):
    current_control = vehicle.get_control()
    steer = round((current_control.steer + steer) * 0.5, 5)
    throttle = round(throttle, 3)
    brake = round(brake, 3)
    control = carla.VehicleControl(steer=steer, throttle=throttle, brake=brake)
    vehicle.apply_control(control)

def destroy_all_vehicles_traffics(world, vehicle_flag=True, traffic_flag=True):
    actors = []
    if vehicle_flag:
        actors += list(world.get_actors().filter('*vehicle*'))
    if traffic_flag:
        actors += list(world.get_actors().filter("*prop*"))
    for actor in actors:
        actor.destroy()

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

    def create_background_vehicle(self, location):
        vehicle_model = "vehicle.tesla.model3" 
        blueprint_car = self.blueprint_library.filter('*vehicle*')
        cars = [bp for bp in blueprint_car if 'model3' in bp.id.lower()]
        
        transform = carla.Transform(location)
        vehicle = self.world.try_spawn_actor(random.choice(cars), transform)
        if vehicle:
            vehicle.set_autopilot(True)  # Set to autopilot for autonomous driving
            print("Background vehicle spawned and set to autopilot.")
        else:
            print("Failed to spawn background vehicle.")
        return vehicle

    def attach_collision_sensor(self, vehicle, callback):
        blueprint = self.blueprint_library.find('sensor.other.collision')
        collision_sensor = self.world.spawn_actor(blueprint, carla.Transform(), attach_to=vehicle)
        collision_sensor.listen(callback)
        return collision_sensor
    
class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: IMUSensor._on_imu_event(weak_self, event))

    @staticmethod
    def _on_imu_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.course_angle = event.compass

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

class Main_Car_Control:
    def __init__(self, main_car, world, window, instantaneous_speed=False):
        self.vehicle = main_car
        self.instantaneous_speed = instantaneous_speed
        self.world = world
        self.autopilot_flag = True
        self.speed_limit = 100
        self.steer = 0
        self.throttle = 0
        self.brake = 0
        self.window = window
        self.collision_occurred = False
        self.start_time = time.time()
        self.collision_time = None
        self.running = True
        self.takeover_triggered = False
        self.takeover_status  = False
        self.background_vehicle = None
        self.background_gnss_sensor = None

        self.gnss_sensor = GnssSensor(self.vehicle)  # Initialize GNSS sensor
        self.imu_sensor = IMUSensor(self.vehicle)    # Initialize IMU sensor
        self.vehicle_data =  [{
                    "frameId": 0,
                    "name": "Main_car",
                    "id": 0,
                    "isPlayer": 1,
                    "vehicleColor": 0,
                    "vehicleType": 1,
                    "length": 1,
                    "width": 1,
                    "height": 1,
                    "longitude": 0,
                    "latitude": 0,
                    "courseAngle": 0,
                    "speed": 0,
                    "Av": 1,
                    "alt": 0,
                    "pitchrate": 0,
                    "rollrate": 0,
                    "yawrate": 0,
                    "pitch": 0,
                    "roll": 0,
                    "accx": 0,
                    "accy": 0,
                    "accz": 0,
                    "speedN": 0,
                    "speedE": 0,
                    "speedD": 0
                }]

    def follow_road(self):
        if not self.background_vehicle:
            self.background_vehicle = vehicle_traffic.create_background_vehicle(background_vehicle_location)
            self.background_gnss_sensor = GnssSensor(self.background_vehicle)
        while self.running:
            self.speed = self.get_speed()
            speed["speed"] = self.speed
            self.window.speed = self.speed
            self.steerCmd, self.acc, self.brake = get_sensor_data()
            self.check_takeover_conditions()

            # Update vehicle data
            self.vehicle_data[0]["latitude"] = self.gnss_sensor.lat
            self.vehicle_data[0]["longitude"] = self.gnss_sensor.lon
            self.vehicle_data[0]["courseAngle"] = 95 + self.imu_sensor.course_angle 
            self.vehicle_data[0]["speed"] = self.speed
            self.vehicle_data[0]["accx"] = self.acc

            if int(time.time() * 10) % 10 == 0:
                VehicleDataPublisher.add_data(self.vehicle_data)  

            if self.autopilot_flag == False:
                car_control(self.vehicle, self.steerCmd, self.acc, self.brake)
                # print( self.steerCmd, self.acc, self.brake)
    # def check_finish(self):

    def check_takeover_conditions(self):
        location = self.vehicle.get_location()
        if location.x > -195 and not self.takeover_triggered:
            print("Takeover Request")
            send_mqtt_message()
            self.takeover_triggered = True
            

        if self.brake > 0.5 and not self.takeover_status:
            print("Takeover success!")
            self.takeover_status = True
            self.vehicle.set_autopilot(False)
            self.autopilot_flag = False

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
            collision_message = "Collision!"
            self.window.set_collision_info(collision_message)
            self.speed = 0
            self.stop_scenario()  

    def get_speed(self):
        velocity = self.vehicle.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        return int(speed * 3.6)


class Window:
    def __init__(self, world, blueprint_library, vehicle):
        self.world = world
        self.vehicle = vehicle
        # self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1920, 360
        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 5760, 1080
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

class VehicleDataPublisher(threading.Thread):
    data_queue = []

    @staticmethod
    def add_data(vehicle_data):
        VehicleDataPublisher.data_queue.append(vehicle_data)

    def run(self):
        while True:
            if VehicleDataPublisher.data_queue:
                data_to_publish = VehicleDataPublisher.data_queue.pop(0)
                send_to_redis(data_to_publish)
            time.sleep(0.1)

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
    main_car_control = Main_Car_Control(vehicle, world, window, True)
    collision_sensor = vehicle_traffic.attach_collision_sensor(vehicle, main_car_control.collision_event)

    publisher_thread = VehicleDataPublisher()
    publisher_thread.daemon = True
    publisher_thread.start()

    threading.Thread(target=main_car_control.follow_road).start()

    while True:
        world.tick()
        time.sleep(0.01)
