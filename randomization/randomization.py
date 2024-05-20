import numpy as np
from omni.isaac.core.prims import RigidPrimView
from pxr import UsdPhysics, Gf

class RandomizationObserver:
  pass

class RandomizationNotifier:
  def __init__ (self, observers: list[RandomizationObserver]):
    self.observers = observers

  def notify (self):
    for observer in self.observers:
      observer.update(self)

class RandomizationObserver:
  def update (self, notifier: RandomizationNotifier):
    pass

class RandomizationNotifierToObserverAdapter (RandomizationObserver):
  def __init__ (self, notifier: RandomizationNotifier):
    super().__init__()
    self.notifier = notifier
  
  def update (self, notifier: RandomizationNotifier):
    self.notifier.notify()
