from quadcopter_system.sensor.imu_sensor import ImuSensor
from quadcopter_system.sensor.euler_state import EulerState
from omni.isaac.core.prims import RigidPrimView
from quadcopter_system.control_system import QuadcopterControlSystem

class Actuator:
  def __init__ (self, quadcopters: RigidPrimView, imu_sensor: ImuSensor, euler_state: EulerState, control_system: QuadcopterControlSystem):
    self.quadcopters = quadcopters
    self.imu_sensor = imu_sensor
    self.euler_state = euler_state
    self.control_system = control_system

  def update (self, ref):
    imu_state = self.imu_sensor.cur_state
    phi = self.euler_state.phi
    phi_dot = self.euler_state.phi_dot
    self.control_system.calculate
