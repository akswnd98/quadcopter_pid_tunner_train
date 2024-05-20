import numpy as np
from omni.isaac.kit import SimulationApp
import os

CONFIG = {
  "headless": False,
}
simulation_app = SimulationApp(launch_config=CONFIG)


from omni.isaac.core import World

physics_dt = 1.0 / 1000.0
rendering_dt = 1.0 / 30.0
xn = 4
yn = 4
assets_root_path = 'omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/'
quadcopter_usd_path = os.path.join(assets_root_path, 'Isaac/Robots/Quadcopter/quadcopter.usd')

world = World(stage_units_in_meters=1.0)


import torch

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


from env.env import QuadcopterOriginPositionsGenerator, generate_pid_tunners, initialize_env

position_generator = QuadcopterOriginPositionsGenerator(xn, yn)
positions = position_generator.generate()

pid_tunners = generate_pid_tunners(positions)
initialize_env(world, pid_tunners, physics_dt, rendering_dt)

from omni.isaac.core.prims import RigidPrimView
from randomization.randomization import RandomizationNotifier, RandomizationNotifierToObserverAdapter
from randomization.scale import RandomScaleModel, RandomizeScale
from randomization.mass import RandomMassModel, RandomizeMass
from randomization.inertia import RandomInertiaModel, RandomizeInertia
from randomization.signal import RandomSignalModel
from quadcopter_system.sensor.sensor import ContainerSensor
from quadcopter_system.sensor.imu_sensor import ImuSensor
from quadcopter_system.sensor.euler_state import EulerState, ZeroInitialization
from quadcopter_system.control_system import QuadcopterControlSystem

quadcopters = RigidPrimView('/World/PidTunner_*/quadcopter')
quadcopters.enable_rigid_body_physics()

random_scale_model = RandomScaleModel(quadcopters, [RandomizeScale(quadcopters)], xn * yn)
random_mass_model = RandomMassModel(quadcopters, [RandomizeMass(quadcopters)], xn * yn)
random_inertia_model = RandomInertiaModel(quadcopters, [RandomizeInertia(quadcopters)], xn * yn)
random_signal_model = RandomSignalModel(xn * yn, random_mass_model)

randomizer = RandomizationNotifier([
  RandomizationNotifierToObserverAdapter(random_scale_model),
  RandomizationNotifierToObserverAdapter(random_mass_model),
  RandomizationNotifierToObserverAdapter(random_inertia_model),
  RandomizationNotifierToObserverAdapter(random_signal_model)
])

imu_sensor = ImuSensor(xn * yn)
euler_state = EulerState(imu_sensor, ZeroInitialization(xn * yn))

sensor_container = ContainerSensor([
  imu_sensor,
  euler_state
])

control_system = QuadcopterControlSystem(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, random_signal_model, imu_sensor, euler_state, xn * yn)

class PhysicsStepHandler:
  def __init__ (self):
    self.cnt = 0
  
  def get_physics_step_handler (self):
    return lambda step_size: self.handle_physics_step(step_size)

  def handle_physics_step (self, step_size):
    sensor_container.update()
    control_system.update()
    if self.cnt >= 1000:
      print('euler_state.phi[0] euler_state.phi_dot[0]: ', euler_state.phi[0], euler_state.phi_dot[0])
      self.cnt = 0
    else:
      self.cnt += 1

physics_step_handler = PhysicsStepHandler()

world.add_physics_callback('main', physics_step_handler.get_physics_step_handler())

while True:
  randomizer.notify()

  imu_sensor = ImuSensor(xn * yn)

  world.reset()
  for _ in range(6000):
    world.step()
  world.stop()

simulation_app.close()
