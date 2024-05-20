import os
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.cloner import Cloner
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.articulations import ArticulationView
import numpy as np
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core import World
from typing import TypedDict
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import RigidPrimView, XFormPrim, XFormPrimView
from pxr import UsdPhysics
import omni
from pxr import Gf
from pxr.UsdPhysics import RigidBodyAPI
from pxr.UsdGeom import Cube
from pxr.UsdGeom import XformCommonAPI
from pxr.UsdPhysics import ArticulationRootAPI

class QuadcopterOriginPositionsGenerator:
  def __init__ (self, xn: int, yn: int):
    self.xn = xn
    self.yn = yn
  
  def generate (self):
    ret: np.ndarray = np.zeros((self.xn * self.yn, 3), dtype=np.float32)
    for y in range(self.yn):
      for x in range(self.xn):
        ret[y * self.xn + x][0] = 8 * x
        ret[y * self.xn + x][1] = 8 * y
        ret[y * self.xn + x][2] = 0
    
    return ret

def generate_pid_tunners (pid_tunner_origin_positions: np.ndarray[np.ndarray[np.float32]]):
  assets_root_path = 'omniverse://localhost/Projects/'
  pid_tunner_usd_path = os.path.join(assets_root_path, 'pid_tunner.usd')
  add_reference_to_stage(pid_tunner_usd_path, '/World/PidTunner_0')

  Articulation(prim_path='/World/PidTunner_0', name='pid_tunner')

  RigidBodyAPI.Apply(omni.usd.get_context().get_stage().GetPrimAtPath('/World/PidTunner_0/stator')).CreateSimulationOwnerRel().SetTargets(['/physicsScene'])
  RigidBodyAPI.Apply(omni.usd.get_context().get_stage().GetPrimAtPath('/World/PidTunner_0/quadcopter')).CreateSimulationOwnerRel().SetTargets(['/physicsScene'])

  cloner = Cloner()
  cloner.filter_collisions('/physicsScene', '/World', list(map(lambda x: '/World/PidTunner_{}'.format(x), list(range(0, pid_tunner_origin_positions.shape[0])))))
  target_paths = cloner.generate_paths("/World/PidTunner", pid_tunner_origin_positions.shape[0])
  cloner.clone(source_prim_path="/World/PidTunner_0", prim_paths=target_paths, positions=pid_tunner_origin_positions)
  pid_tunners = ArticulationView('/World/PidTunner_*', 'pid_tunner_view')

  return pid_tunners

'''def generate_pid_tunners (pid_tunner_origin_positions: np.ndarray[np.ndarray[np.float32]]):
  pid_tunner = XFormPrim('/World/PidTunner_0', 'pid_tunner_0')
  stator = DynamicCuboid(prim_path='/World/PidTunner_0/stator', name='stator', color=np.array([0.0, 1.0, 0.0]))
  quadcopter = DynamicCuboid(prim_path='/World/PidTunner_0/quadcopter', name='quadcopter', translation=np.array([3.0, 0.0, 0.0]), scale=np.array([2.0, 2.0, 0.1]), color=np.array([0.0, 1.0, 0.0]))
  omni.usd.get_context().get_stage().GetPrimAtPath('/World/PidTunner_0/quadcopter')

  fixed_joint = UsdPhysics.FixedJoint.Define(
    omni.usd.get_context().get_stage(),
    '/World/PidTunner_0/stator/fixedjoint'
  )

  fixed_joint.CreateBody0Rel().SetTargets(['/World/PidTunner_0'])
  fixed_joint.CreateBody1Rel().SetTargets(['/World/PidTunner_0/stator'])
  pos = Gf.Vec3f([0.0, 0.0, 0.0])
  fixed_joint.CreateLocalPos1Attr().Set(pos)
  fixed_joint.CreateJointEnabledAttr(True)

  stator_rigidbody = RigidBodyAPI.Apply(omni.usd.get_context().get_stage().GetPrimAtPath('/World/PidTunner_0/stator'))
  stator_rigidbody.CreateRigidBodyEnabledAttr(True)
  stator_rigidbody.CreateSimulationOwnerRel().SetTargets(['/physicsScene'])

  quadcopter_rigidbody = RigidBodyAPI.Apply(omni.usd.get_context().get_stage().GetPrimAtPath('/World/PidTunner_0/quadcopter'))
  quadcopter_rigidbody.CreateRigidBodyEnabledAttr(True)
  quadcopter_rigidbody.CreateSimulationOwnerRel().SetTargets(['/physicsScene'])

  revolute_joint = UsdPhysics.RevoluteJoint.Define(omni.usd.get_context().get_stage(), '/World/PidTunner_0/quadcopter/revolutejoint')
  revolute_joint.CreateBody0Rel().SetTargets(['/World/PidTunner_0/stator'])
  revolute_joint.CreateBody1Rel().SetTargets(['/World/PidTunner_0/quadcopter'])
  revolute_joint.CreateLocalPos0Attr().Set(Gf.Vec3f([1.0, 0.0, 0.0]))
  revolute_joint.CreateLocalPos1Attr().Set(Gf.Vec3f([-1.0, 0.0, 0.0]))
  revolute_joint.CreateJointEnabledAttr(True)

  ArticulationRootAPI.Apply(omni.usd.get_context().get_stage().GetPrimAtPath('/World/PidTunner_0'))

  cloner = Cloner()
  cloner.filter_collisions('/physicsScene', '/World', list(map(lambda x: '/World/PidTunner_{}'.format(x), list(range(0, pid_tunner_origin_positions.shape[0])))))
  target_paths = cloner.generate_paths("/World/PidTunner", pid_tunner_origin_positions.shape[0])
  cloner.clone(source_prim_path="/World/PidTunner_0", prim_paths=target_paths, positions=pid_tunner_origin_positions)
  pid_tunners = XFormPrimView('/World/PidTunner_*', 'pid_tunner_view')

  # XformCommonAPI(omni.usd.get_context().get_stage().GetPrimAtPath('/World/PidTunner_1/quadcopter')).SetTranslate(Gf.Vec3d([3.0, 0.0, 0.0]))

  return pid_tunners'''

def initialize_env (world: SimulationContext, pid_tunners: ArticulationView, physics_dt: float, rendering_dt: float):
  world.set_simulation_dt(physics_dt, rendering_dt)
  world.scene.add(pid_tunners)

class PhysicsConstant:
  g = 9.81
