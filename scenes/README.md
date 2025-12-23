# Scenes Directory
# 장면 디렉토리

This directory contains USD (Universal Scene Description) files for the IP equipment simulation.

이 디렉토리는 IP 장비 시뮬레이션을 위한 USD(Universal Scene Description) 파일을 포함합니다.

## USD Files / USD 파일

Place your USD model files here for use in the simulation.

시뮬레이션에서 사용할 USD 모델 파일을 여기에 배치하세요.

### Example / 예시

```
scenes/
├── ip_equipment.usd       # Main equipment model / 주요 장비 모델
├── environment.usd        # Environment assets / 환경 에셋
└── README.md
```

## Creating USD Models / USD 모델 생성

### Method 1: Isaac Sim / Isaac Sim 사용

1. Open Isaac Sim
2. Create or import your 3D model
3. Save as USD file: File > Save As > USD

### Method 2: Convert from Other Formats / 다른 형식에서 변환

Isaac Sim supports importing various formats:
- URDF (Robot models)
- STL
- OBJ
- FBX

Use Isaac Sim's import tools to convert to USD.

### Method 3: Programmatic Creation / 프로그래밍 방식 생성

Create USD files programmatically using Python:

```python
from pxr import Usd, UsdGeom

# Create a new USD stage
stage = Usd.Stage.CreateNew('my_equipment.usd')

# Add geometry
xform = UsdGeom.Xform.Define(stage, '/World')
cube = UsdGeom.Cube.Define(stage, '/World/Cube')

# Save
stage.Save()
```

## Using USD Files in Simulation / 시뮬레이션에서 USD 파일 사용

Update your configuration file to reference the USD model:

```yaml
equipment:
  model_path: scenes/ip_equipment.usd
  position: [0.0, 0.0, 1.0]
  orientation: [0.0, 0.0, 0.0, 1.0]
```

## Best Practices / 모범 사례

1. **Naming**: Use descriptive names for USD files
2. **Organization**: Group related assets together
3. **Scale**: Ensure models are at correct scale (meters in Isaac Sim)
4. **Origin**: Set origin point appropriately for equipment
5. **Materials**: Include materials and textures in the USD file

## Resources / 참고 자료

- [USD Documentation](https://graphics.pixar.com/usd/docs/index.html)
- [Isaac Sim USD Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html)
- [Omniverse Asset Converter](https://docs.omniverse.nvidia.com/services/latest/asset-converter.html)
