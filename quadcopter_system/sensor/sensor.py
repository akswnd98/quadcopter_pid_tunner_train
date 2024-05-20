class Sensor:
  def update (self):
    pass

class ContainerSensor (Sensor):
  def __init__ (self, sensors: list[Sensor]):
    self.sensors = sensors

  def update (self):
    for sensor in self.sensors:
      sensor.update()
