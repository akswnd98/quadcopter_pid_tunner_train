import numpy as np
from env.env import PhysicsConstant

def generate_xy_symmetrical_random_vec3 (env_num: int, xy_lower_bound: float, xy_upper_bound: float, z_lower_bound: float, z_upper_bound: float):
  xy_log_lower_bound = np.log(xy_lower_bound)
  xy_log_upper_bound = np.log(xy_upper_bound)
  xy_log_random_value = np.random.uniform(xy_log_lower_bound, xy_log_upper_bound, size=(env_num, 1))
  xy_random_value = np.exp(xy_log_random_value)

  z_log_lower_bound = np.log(z_lower_bound)
  z_log_upper_bound = np.log(z_upper_bound)
  z_log_random_value = np.random.uniform(z_log_lower_bound, z_log_upper_bound, size=(env_num, 1))
  z_random_value = np.exp(z_log_random_value)

  return np.concatenate([np.array(xy_random_value), np.array(xy_random_value), z_random_value], axis=1)

def generate_random_scalar (env_num: int, lower_bound: float, upper_bound: float):
  log_lower_bound = np.log(lower_bound)
  log_upper_bound = np.log(upper_bound)
  log_random_value = np.random.uniform(log_lower_bound, log_upper_bound, size=(env_num, ))

  return np.exp(log_random_value)

def generate_random_signal_thrust_ratio (env_num: int, signal_max: np.ndarray, masses: np.ndarray):
  lower_bound = masses * PhysicsConstant.g / 2 / signal_max
  upper_bound = masses * PhysicsConstant.g / signal_max
  return generate_random_scalar(env_num, lower_bound, upper_bound)
