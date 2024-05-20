from quadcopter_system.sensor.sensor import Sensor
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.sensor import _sensor
import numpy as np

class ImuSensor (Sensor):
  def __init__ (self, quadcopter_num: int):
    super().__init__()
    self.quadcopter_num = quadcopter_num
    self.imu_sensor_interface = _sensor.acquire_imu_sensor_interface()
    self.cur_state = np.array([[0] * 10] * self.quadcopter_num)
  
  def update(self):
    imu_sensor_readings = [self.imu_sensor_interface.get_sensor_reading('/World/PidTunner_{}/quadcopter/imuSensor'.format(i)) for i in range(self.quadcopter_num)]
    prev_state = self.cur_state
    self.cur_state = np.array([
      [
        imu_sensor_readings[i].lin_acc_x,
        imu_sensor_readings[i].lin_acc_y,
        imu_sensor_readings[i].lin_acc_z,
        imu_sensor_readings[i].ang_vel_x,
        imu_sensor_readings[i].ang_vel_y,
        imu_sensor_readings[i].ang_vel_z,
        imu_sensor_readings[i].ang_vel_x - prev_state[i, 3],
        imu_sensor_readings[i].ang_vel_y - prev_state[i, 4],
        imu_sensor_readings[i].ang_vel_z - prev_state[i, 5],
        imu_sensor_readings[i].orientation[0],
        imu_sensor_readings[i].orientation[1],
        imu_sensor_readings[i].orientation[2],
        imu_sensor_readings[i].orientation[3],
      ] for i in range(self.quadcopter_num)
    ])
