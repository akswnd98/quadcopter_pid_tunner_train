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

class RandomInertiaModel (RandomizationNotifier):
  def __init__ (self, quadcopters: RigidPrimView, observers: list[RandomizationObserver], env_num):
    super().__init__(observers)
    self.quadcopters = quadcopters
    self.inertias = np.zeros((env_num, 3), dtype=np.float32)
    self.env_num = env_num

  def notify (self):
    self.inertias = generate_xy_symmetrical_random_vec3(
      self.env_num,
      0.1 * 0.2 ** 2 / 12,
      10 * 1 ** 2 / 12,
      0.1 * 0.2 ** 2 / 6,
      10 * 1 ** 2 / 6
    )
    super().notify()

class RandomizeInertia (RandomizationObserver):
  def __init__ (self, quadcopters: RigidPrimView):
    self.quadcopters = quadcopters

  def update (self, notifier: RandomInertiaModel):
    for quadcopter, inertia in zip(self.quadcopters.prims, notifier.inertias):
      quadcopter_mass_api = UsdPhysics.MassAPI.Apply(quadcopter)
      quadcopter_mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia.tolist()))
