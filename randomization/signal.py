from randomization.randomization import (
  RandomizationNotifier,
)
from randomization.utils import (
  generate_random_scalar,
  generate_random_signal_thrust_ratio,
)
from env.env import PhysicsConstant
import numpy as np

class RandomSignalModel (RandomizationNotifier):
  def __init__ (self, env_num, random_mass_model):
    super().__init__([])
    self.env_num = env_num
    self.random_mass_model = random_mass_model

  def notify (self):
    self.random_signal_max = generate_random_scalar(self.env_num, 1000, 20000)
    self.random_signal_thrust_ratio = generate_random_signal_thrust_ratio(self.env_num, self.random_signal_max, self.random_mass_model.masses)
    # self.signal_level * self.random_signal_thrust_ratio * 4 = self.random_mass_model.masses * g
    self.signal_level = self.random_mass_model.masses * PhysicsConstant.g / (self.random_signal_thrust_ratio * 4)
    super().notify()
  
  def get_saturated_signal (self, signal):
    min_saturated = np.max(
      np.concatenate([
        np.expand_dims(signal, axis=-1),
        np.expand_dims(np.zeros_like(signal), axis=-1)
      ], axis=-1),
      axis=-1
    )
    return np.min(
      np.concatenate([
        np.expand_dims(min_saturated, axis=-1),
        np.expand_dims(np.ones_like(min_saturated) * np.expand_dims(self.random_signal_max, axis=-1), axis=-1)
      ], axis=-1),
      axis=-1
    )
  
  def get_thrust (self, delta_signal):
    return self.get_saturated_signal(
      np.expand_dims(self.signal_level, axis=-1) + delta_signal
    ) * np.expand_dims(self.random_signal_thrust_ratio, axis=-1)
