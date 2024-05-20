from quadcopter_system.sensor.sensor import Sensor
from quadcopter_system.sensor.imu_sensor import ImuSensor
import numpy as np

class EulerStateInitialization:
  def generate_init_state (self):
    pass

class ZeroInitialization (EulerStateInitialization):
  def __init__ (self, env_num: int):
    self.env_num = env_num

  def generate_init_state (self):
    return {
      'phi': np.array([0] * self.env_num),
      'theta': np.array([0] * self.env_num),
      'phi_dot': np.array([0] * self.env_num),
      'theta_dot': np.array([0] * self.env_num),
      'psi_dot': np.array([0] * self.env_num)
    }

class EulerState (Sensor):
  def __init__ (self, imu_sensor: ImuSensor, initialization: EulerStateInitialization):
    self.imu_sensor = imu_sensor
    init_state = initialization.generate_init_state()
    self.phi = init_state['phi']
    self.theta = init_state['theta']
    self.phi_dot = init_state['phi_dot']
    self.theta_dot = init_state['theta_dot']
    self.psi_dot = init_state['psi_dot']
    self.dt = 0.001
    self.alpha = 0.996
  
  def update (self):
    imu_state = self.imu_sensor.cur_state
    phi_hf = self.phi + self.phi_dot * self.dt
    theta_hf = self.theta + self.theta_dot * self.dt
    phi_lf = np.arctan(imu_state[:, 1] / imu_state[:, 2])
    theta_lf = np.arcsin(imu_state[:, 0] / np.sqrt(imu_state[:, 0] ** 2 + imu_state[:, 1] ** 2 + imu_state[:, 2] ** 2))
    self.phi = phi_hf * self.alpha + phi_lf * (1.0 - self.alpha)
    self.theta = theta_hf * self.alpha + theta_lf * (1.0 - self.alpha)
    C_inv = [np.array([
      [1, np.sin(phi_i) * np.tan(theta_i), np.cos(phi_i) * np.tan(theta_i)],
      [0, np.cos(phi_i), -np.sin(phi_i)],
      [0, np.sin(phi_i) / np.cos(theta_i), np.cos(phi_i) / np.cos(theta_i)]
    ]) for phi_i, theta_i in zip(self.phi, self.theta)]
    eta_dot = np.array([C_inv_i @ w_i for C_inv_i, w_i in zip(C_inv, imu_state[:, 3: 6])])
    self.phi_dot = eta_dot[:, 0]
    self.theta_dot = eta_dot[:, 1]
    self.psi_dot = eta_dot[:, 2]
