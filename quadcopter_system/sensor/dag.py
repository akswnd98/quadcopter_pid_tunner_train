from quadcopter_system.sensor.sensor import Sensor

class SensorDAGNode:
  pass

class SensorDAGFactory:
  def generate_sensor_nodes (self):
    pass

  def generate_sensor_edges (self):
    pass

class SensorDAG:
  def __init__ (self, sensor_nodes: list[SensorDAGNode], sensor_edges: list[tuple[SensorDAGNode, SensorDAGNode]]):
    self.sensor_nodes = sensor_nodes
    self.sensor_edges = sensor_edges
    for sensor_edge in self.sensor_edges:
      sensor_edge[0].next_nodes.append(sensor_edges[1])
      sensor_edges[1].indegree += 1

    self.initial_queue = []
    for sensor_node in self.sensor_nodes:
      if sensor_node.indgree <= 0:
        self.initial_queue.append(sensor_node)
    
    for sensor_node in self.sensor_nodes:
      sensor_node.initial_indegree = sensor_node.indegree
  
  def update (self):
    self.queue = self.initial_queue.copy()
    for sensor_node in self.sensor_nodes:
      sensor_node.indegree = sensor_node.initial_indegree
    while len(self.queue) > 0:
      self.queue[0].update()
      for next_node in self.queue[0].next_nodes:
        next_node.indegree -= 1
        if next_node.indegree <= 0:
          self.queue.append(next_node)
      self.queue.pop(0)
    
class SensorDAGByFactory (SensorDAG):
  def __init__ (self, factory: SensorDAGFactory):
    super().__init__(factory.generate_sensor_nodes(), factory.generate_sensor_edges())

class SensorDAGNode:
  def __init__ (self, sensor: Sensor):
    self.sensor = sensor
    self.indegree = 0
    self.next_nodes: list[SensorDAGNode] = []
    self.initial_indegree = 0

  def update (self):
    self.sensor.update()
