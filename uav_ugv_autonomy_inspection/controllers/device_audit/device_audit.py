"""Webots controller: enumerate devices before connecting motion control."""
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())
print("\n=== WEBOTS DEVICE AUDIT ===")
print("Robot name:", robot.getName())
print("Basic timestep (ms):", timestep)
print("Device count:", robot.getNumberOfDevices())
for i in range(robot.getNumberOfDevices()):
    dev = robot.getDeviceByIndex(i)
    try:
        typ = dev.getNodeType()
    except Exception:
        typ = "(type unavailable)"
    print(f"[{i:02d}] name={dev.getName()!r} type={typ}")
print("=== END DEVICE AUDIT ===\n")
while robot.step(timestep) != -1:
    pass
