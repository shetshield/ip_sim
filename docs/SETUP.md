# IP Equipment Isaac Sim Setup Guide
# IP 장비 Isaac Sim 설치 가이드

## Prerequisites / 사전 요구사항

### System Requirements / 시스템 요구사항
- Ubuntu 20.04 or 22.04 (recommended)
- NVIDIA GPU with driver version 470 or higher
- 32GB RAM (minimum), 64GB recommended
- 50GB free disk space

### Software Requirements / 소프트웨어 요구사항
- Isaac Sim 2023.1.0 or later
- Python 3.7 or later
- NVIDIA Omniverse Launcher

## Installation / 설치

### 1. Install Isaac Sim / Isaac Sim 설치

1. Download and install NVIDIA Omniverse Launcher from:
   https://www.nvidia.com/en-us/omniverse/

2. Launch Omniverse Launcher and install Isaac Sim from the Exchange tab

3. Verify installation by launching Isaac Sim

### 2. Setup Project / 프로젝트 설정

```bash
# Clone the repository
git clone https://github.com/shetshield/ip_sim.git
cd ip_sim

# Install Python dependencies
pip install -r requirements.txt
```

### 3. Configure Isaac Sim Python Environment / Isaac Sim Python 환경 설정

Isaac Sim comes with its own Python environment. You can use it by:

```bash
# Find your Isaac Sim installation path
# Default: ~/.local/share/ov/pkg/isaac_sim-*

# Set environment variable
export ISAAC_SIM_PATH=~/.local/share/ov/pkg/isaac_sim-2023.1.0

# Use Isaac Sim's Python
alias isaac_python="${ISAAC_SIM_PATH}/python.sh"
```

## Configuration / 설정

Edit the configuration file at `config/default_config.yaml` to customize:
- Simulation parameters (timestep, duration)
- Scene setup (ground plane, lighting, camera)
- Equipment properties (type, position, orientation)
- Physics settings (gravity, physics engine)
- Controller parameters

## Running the Simulation / 시뮬레이션 실행

### Method 1: Using Scripts / 스크립트 사용

```bash
# Run with GUI
./scripts/run_gui.sh

# Run in headless mode (no GUI)
./scripts/run_headless.sh
```

### Method 2: Direct Python Execution / 직접 Python 실행

```bash
# With default configuration
python3 scripts/run_simulation.py

# With custom configuration
python3 scripts/run_simulation.py --config config/default_config.yaml

# Headless mode
python3 scripts/run_simulation.py --headless

# Run for specific number of steps
python3 scripts/run_simulation.py --steps 1000
```

### Method 3: Using Isaac Sim Python / Isaac Sim Python 사용

```bash
# Set ISAAC_SIM_PATH first (see above)
isaac_python scripts/run_simulation.py --config config/default_config.yaml
```

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

## Troubleshooting / 문제 해결

### Issue: Isaac Sim modules not found
**Solution**: Make sure you're using Isaac Sim's Python environment:
```bash
export ISAAC_SIM_PATH=~/.local/share/ov/pkg/isaac_sim-2023.1.0
${ISAAC_SIM_PATH}/python.sh scripts/run_simulation.py
```

### Issue: GPU not detected
**Solution**: Check NVIDIA driver installation:
```bash
nvidia-smi
```

### Issue: Permission denied for scripts
**Solution**: Make scripts executable:
```bash
chmod +x scripts/*.sh
```

## Next Steps / 다음 단계

1. Read the [Usage Guide](USAGE.md) for detailed simulation usage
2. Customize the configuration in `config/default_config.yaml`
3. Add your custom equipment USD models to the `scenes/` directory
4. Extend the simulation code in `src/ip_sim/` for your specific needs

## Support / 지원

For issues and questions, please create an issue on the GitHub repository.
