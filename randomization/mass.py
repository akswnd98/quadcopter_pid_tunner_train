from randomization.randomization import (
  RandomizationNotifier,
  RandomizationObserver,
  RandomizationNotifierToObserverAdapter
)
from omni.isaac.core.prims import RigidPrimView
import numpy as np
from randomization.utils import (
  generate_random_scalar,
  generate_random_signal_thrust_ratio,
  generate_xy_symmetrical_random_vec3
)
from pxr import UsdPhysics
from pxr import Gf

class RandomMassModel (RandomizationNotifier):
  def __init__ (self, quadcopters: RigidPrimView, observers: list[RandomizationObserver], env_num):
    super().__init__(observers)
    self.quadcopters = quadcopters
    self.inertias = np.zeros((env_num, ), dtype=np.float32)
    self.env_num = env_num

  def notify (self):
    self.masses = generate_random_scalar(self.env_num, 0.1, 10)
    super().notify()

class RandomizeMass (RandomizationObserver):
  def __init__ (self, quadcopters: RigidPrimView):
    self.quadcopters = quadcopters

  def update (self, notifier: RandomMassModel):
    for quadcopter, mass in zip(self.quadcopters.prims, notifier.masses):
      quadcopter_mass_api = UsdPhysics.MassAPI.Apply(quadcopter)
      quadcopter_mass_api.CreateMassAttr(mass)
