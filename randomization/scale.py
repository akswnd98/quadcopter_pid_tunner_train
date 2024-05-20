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
from pxr import UsdGeom
from pxr.UsdGeom import Xform, XformCommonAPI, Xformable
import omni
from pxr import Gf

class RandomScaleModel (RandomizationNotifier):
  def __init__ (self, quadcopters: RigidPrimView, observers: list[RandomizationObserver], env_num):
    super().__init__(observers)
    self.quadcopters = quadcopters
    self.scales = np.zeros((env_num, 3), dtype=np.float32)
    self.env_num = env_num

  def notify (self):
    self.scales = generate_xy_symmetrical_random_vec3(self.env_num, 0.2, 1, 0.01, 0.01)
    super().notify()

class RandomizeScale (RandomizationObserver):
  def __init__ (self, quadcopters: RigidPrimView):
    self.quadcopters = quadcopters

  def update (self, notifier: RandomScaleModel):
    self.quadcopters.set_local_scales(notifier.scales)
    # for i, quadcopter in zip(range(len(self.quadcopters.prims)), self.quadcopters.prims):
      # quadcopter.GetAttribute('xformOp:scale').Set(Gf.Vec3f(notifier.scales[i].tolist()))
      # XformCommonAPI.Get(omni.usd.get_context().get_stage(), quadcopter_path).SetScale(Gf.Vec3f(notifier.scales[i].tolist()), 0)
