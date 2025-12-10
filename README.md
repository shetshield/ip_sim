# IP Equipment Isaac Sim Simulation
# IP 장비 Isaac Sim 시뮬레이션

로봇-장비 4차년도 IP 장비 Isaac Sim 시뮬레이션 프로젝트

## Overview / 개요

This project provides an Isaac Sim simulation environment for IP (Intellectual Property) equipment as part of the Robot-Equipment Year 4 project. The simulation is built on NVIDIA Isaac Sim, providing physics-based simulation capabilities for testing and development.

이 프로젝트는 로봇-장비 4차년도 프로젝트의 일환으로 IP(지적재산권) 장비를 위한 Isaac Sim 시뮬레이션 환경을 제공합니다. NVIDIA Isaac Sim을 기반으로 구축되어 테스트 및 개발을 위한 물리 기반 시뮬레이션 기능을 제공합니다.

## Features / 주요 기능

- 🤖 **Equipment Simulation**: Simulate IP equipment in a realistic physics environment
- ⚙️ **Configurable**: Easy configuration through YAML files
- 🎮 **Interactive & Headless**: Support for both GUI and headless modes
- 📊 **Extensible**: Modular design for easy customization
- 🔧 **Isaac Sim Integration**: Built on NVIDIA Isaac Sim platform

## Quick Start / 빠른 시작

### Prerequisites / 사전 요구사항

- Ubuntu 20.04/22.04
- NVIDIA GPU with driver 470+
- Isaac Sim 2023.1.0+
- Python 3.7+

### Installation / 설치

```bash
# Clone the repository / 저장소 복제
git clone https://github.com/shetshield/ip_sim.git
cd ip_sim

# Install dependencies / 의존성 설치
pip install -r requirements.txt
```

### Running / 실행

```bash
# Run with GUI / GUI로 실행
./scripts/run_gui.sh

# Run headless / 헤드리스로 실행
./scripts/run_headless.sh

# Or use Python directly / 또는 Python으로 직접 실행
python3 scripts/run_simulation.py --config config/default_config.yaml
```

## Documentation / 문서

- [Setup Guide / 설치 가이드](docs/SETUP.md) - Detailed installation and setup instructions
- [Usage Guide / 사용 가이드](docs/USAGE.md) - How to use and configure the simulation

## Project Structure / 프로젝트 구조

```
ip_sim/
├── config/              # Configuration files / 설정 파일
│   └── default_config.yaml
├── docs/                # Documentation / 문서
│   ├── SETUP.md
│   └── USAGE.md
├── scenes/              # USD scene files / USD 장면 파일
├── scripts/             # Launcher scripts / 실행 스크립트
│   ├── run_simulation.py
│   ├── run_gui.sh
│   └── run_headless.sh
├── src/                 # Source code / 소스 코드
│   └── ip_sim/
│       ├── __init__.py
│       └── simulation.py
├── requirements.txt     # Python dependencies / Python 의존성
└── README.md
```

## Configuration / 설정

Edit `config/default_config.yaml` to customize simulation parameters:

```yaml
simulation:
  headless: false
  physics_dt: 0.016666667  # 60 Hz

scene:
  ground_plane: true
  lighting: default

equipment:
  type: ip_robot
  position: [0.0, 0.0, 0.0]
  orientation: [0.0, 0.0, 0.0, 1.0]
```

See [Usage Guide](docs/USAGE.md) for detailed configuration options.

## Development / 개발

### Extending the Simulation / 시뮬레이션 확장

```python
from ip_sim.simulation import IPSimulation

class CustomSimulation(IPSimulation):
    def add_custom_equipment(self):
        # Add your custom equipment here
        pass
```

### Adding Custom Equipment / 커스텀 장비 추가

1. Place USD models in `scenes/` directory
2. Update configuration to reference your model
3. Extend `IPSimulation` class as needed

## Requirements / 요구사항

- NVIDIA GPU with RTX series recommended
- Isaac Sim 2023.1.0 or later
- Ubuntu 20.04 or 22.04 (Linux)
- 32GB+ RAM recommended
- 50GB+ free disk space

## Support / 지원

For questions and issues, please create an issue on the GitHub repository.

## License / 라이선스

This project is part of the Robot-Equipment Year 4 project.

## Acknowledgments / 감사의 말

This project uses NVIDIA Isaac Sim for physics simulation and robotics development.
