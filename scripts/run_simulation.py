#!/usr/bin/env python3
"""
Script to run the IP equipment simulation
"""

import sys
import os

# Add src directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from ip_sim.simulation import main

if __name__ == "__main__":
    main()
