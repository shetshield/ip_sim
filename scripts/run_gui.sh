#!/bin/bash
# Run IP simulation with GUI

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONFIG_PATH="${SCRIPT_DIR}/../config/default_config.yaml"

python3 "${SCRIPT_DIR}/run_simulation.py" \
    --config "${CONFIG_PATH}"
