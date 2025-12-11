# IP Equipment Simulation Usage Guide
# IP 장비 시뮬레이션 사용 가이드

## Overview / 개요

This guide explains how to use the IP equipment Isaac Sim simulation for the Year 4 robot-equipment project.

이 가이드는 4차년도 로봇-장비 프로젝트를 위한 IP 장비 Isaac Sim 시뮬레이션 사용법을 설명합니다.

## Basic Usage / 기본 사용법

### Starting the Simulation / 시뮬레이션 시작

```bash
# Simple start with default settings
python3 scripts/run_simulation.py

# With custom configuration
python3 scripts/run_simulation.py --config config/default_config.yaml

# Headless mode (no GUI)
python3 scripts/run_simulation.py --headless

# Run for 1000 simulation steps
python3 scripts/run_simulation.py --steps 1000
```

### Using Shell Scripts / 셸 스크립트 사용

```bash
# GUI mode
./scripts/run_gui.sh

# Headless mode
./scripts/run_headless.sh
```

## Configuration / 설정

### Configuration File Structure / 설정 파일 구조

The `config/default_config.yaml` file contains all simulation parameters:

```yaml
simulation:
  headless: false          # Run without GUI
  physics_dt: 0.016666667  # Physics timestep (60 Hz)
  rendering_dt: 0.016666667 # Render timestep (60 Hz)

scene:
  ground_plane: true       # Enable ground plane
  lighting: default        # Lighting setup
  camera:
    position: [5.0, 5.0, 3.0]
    target: [0.0, 0.0, 0.0]

equipment:
  type: ip_robot           # Equipment type
  position: [0.0, 0.0, 0.0] # Initial position [x, y, z]
  orientation: [0.0, 0.0, 0.0, 1.0] # Quaternion [x, y, z, w]
  scale: 1.0
  model_path: null         # Path to USD model

physics:
  enabled: true
  gravity: [0.0, 0.0, -9.81] # Gravity vector [x, y, z]

controller:
  frequency: 100           # Control frequency (Hz)
  type: position          # Controller type
```

### Modifying Configuration / 설정 수정

1. Copy the default configuration:
   ```bash
   cp config/default_config.yaml config/my_config.yaml
   ```

2. Edit your configuration:
   ```bash
   nano config/my_config.yaml
   ```

3. Run with your configuration:
   ```bash
   python3 scripts/run_simulation.py --config config/my_config.yaml
   ```

## Adding Custom Equipment / 커스텀 장비 추가

### USD Model Integration / USD 모델 통합

1. Place your USD model file in the `scenes/` directory:
   ```bash
   cp /path/to/your/robot.usd scenes/ip_equipment.usd
   ```

2. Update the configuration:
   ```yaml
   equipment:
     model_path: scenes/ip_equipment.usd
     position: [0.0, 0.0, 1.0]
     orientation: [0.0, 0.0, 0.0, 1.0]
   ```

### Programmatic Equipment Creation / 프로그래밍 방식 장비 생성

Extend the `IPSimulation` class in `src/ip_sim/simulation.py`:

```python
def add_custom_equipment(self):
    """Add custom IP equipment"""
    from omni.isaac.core.prims import RigidPrim
    
    # Create custom equipment
    equipment = RigidPrim(
        prim_path="/World/Equipment",
        position=self.config['equipment']['position']
    )
    
    return equipment
```

## Simulation Control / 시뮬레이션 제어

### Running Simulation Steps / 시뮬레이션 스텝 실행

The simulation runs in a loop, stepping through time:

```python
from ip_sim.simulation import IPSimulation

sim = IPSimulation()
sim.initialize()
sim.setup_scene()
sim.add_equipment()

# Run for 1000 steps
sim.run(num_steps=1000)

# Or run indefinitely
sim.run()

sim.cleanup()
```

### Accessing Simulation State / 시뮬레이션 상태 접근

```python
# Get current simulation time
time = sim.world.current_time_step_index

# Get physics time
physics_time = sim.world.get_physics_dt()
```

## Camera Control / 카메라 제어

### Setting Camera Position / 카메라 위치 설정

In configuration file:
```yaml
scene:
  camera:
    position: [5.0, 5.0, 3.0]  # Camera position
    target: [0.0, 0.0, 0.0]     # Look-at target
```

### Runtime Camera Control / 런타임 카메라 제어

When running with GUI:
- **Left Mouse**: Rotate view
- **Middle Mouse**: Pan view
- **Right Mouse + Drag**: Zoom in/out
- **Mouse Wheel**: Zoom
- **F**: Frame selected object

## Data Recording / 데이터 기록

### Recording Simulation Data / 시뮬레이션 데이터 기록

You can extend the simulation to record data:

```python
def run_with_recording(self):
    """Run simulation with data recording"""
    import csv
    
    with open('simulation_data.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'position_x', 'position_y', 'position_z'])
        
        step = 0
        while self.simulation_app.is_running():
            self.world.step(render=True)
            
            # Record data every 10 steps
            if step % 10 == 0:
                # Get equipment position
                position = [0.0, 0.0, 0.0]  # Replace with actual position
                writer.writerow([step, *position])
            
            step += 1
```

## Performance Optimization / 성능 최적화

### Headless Mode / 헤드리스 모드

For faster simulation without visualization:
```bash
python3 scripts/run_simulation.py --headless
```

### Physics Settings / 물리 설정

Adjust timestep for speed/accuracy trade-off:
```yaml
simulation:
  physics_dt: 0.033333333  # 30 Hz (faster)
  # or
  physics_dt: 0.008333333  # 120 Hz (more accurate)
```

## Common Tasks / 일반적인 작업

### Task 1: Test Equipment in Basic Scene
```bash
python3 scripts/run_simulation.py --steps 500
```

### Task 2: Run Long Duration Test
```bash
python3 scripts/run_simulation.py --headless --steps 10000
```

### Task 3: Interactive Development
```bash
# Run with GUI for interactive testing
python3 scripts/run_simulation.py
```

## Integration with ROS / ROS 통합

For ROS integration, you can extend the simulation:

```python
import rospy
from geometry_msgs.msg import Pose

class IPSimulationROS(IPSimulation):
    def __init__(self, config_path=None):
        super().__init__(config_path)
        rospy.init_node('ip_simulation')
        self.pose_pub = rospy.Publisher('/equipment/pose', Pose, queue_size=10)
    
    def publish_pose(self, position, orientation):
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        # ... set orientation
        self.pose_pub.publish(pose)
```

## Troubleshooting / 문제 해결

### Simulation Runs Too Slow
- Enable headless mode
- Increase physics_dt value
- Reduce scene complexity

### Equipment Not Visible
- Check equipment position is above ground (z > 0)
- Verify USD model path is correct
- Check camera position and target

### Physics Behaves Unexpectedly
- Verify gravity settings
- Check equipment mass and collision properties
- Adjust physics_dt for stability

## Advanced Topics / 고급 주제

See the Isaac Sim documentation for:
- Advanced physics simulation
- Sensor simulation (cameras, lidars)
- Robot control and manipulation
- Machine learning integration

## Resources / 참고 자료

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Omniverse Documentation](https://docs.omniverse.nvidia.com/)
- [USD Documentation](https://graphics.pixar.com/usd/docs/index.html)
