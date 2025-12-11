"""
Unit tests for IP Simulation
"""

import unittest
import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from ip_sim.simulation import IPSimulation


class TestIPSimulation(unittest.TestCase):
    """Test cases for IPSimulation class"""
    
    def test_initialization(self):
        """Test simulation initialization with default config"""
        sim = IPSimulation()
        self.assertIsNotNone(sim.config)
        self.assertFalse(sim.is_initialized)
        self.assertIsNone(sim.simulation_app)
    
    def test_default_config(self):
        """Test default configuration values"""
        sim = IPSimulation()
        config = sim.config
        
        # Check simulation config
        self.assertIn('simulation', config)
        self.assertIn('physics_dt', config['simulation'])
        self.assertIn('rendering_dt', config['simulation'])
        
        # Check scene config
        self.assertIn('scene', config)
        self.assertIn('ground_plane', config['scene'])
        
        # Check equipment config
        self.assertIn('equipment', config)
        self.assertIn('type', config['equipment'])
        self.assertIn('position', config['equipment'])
        self.assertIn('orientation', config['equipment'])
    
    def test_config_loading(self):
        """Test loading configuration from file"""
        # Create temporary config
        import tempfile
        import yaml
        
        test_config = {
            'simulation': {
                'headless': True,
                'physics_dt': 0.01,
                'rendering_dt': 0.01,
            },
            'equipment': {
                'type': 'test_robot',
                'position': [1.0, 2.0, 3.0],
            }
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            yaml.dump(test_config, f)
            config_path = f.name
        
        try:
            sim = IPSimulation(config_path=config_path)
            self.assertEqual(sim.config['simulation']['headless'], True)
            self.assertEqual(sim.config['simulation']['physics_dt'], 0.01)
            self.assertEqual(sim.config['equipment']['type'], 'test_robot')
            self.assertEqual(sim.config['equipment']['position'], [1.0, 2.0, 3.0])
        finally:
            os.unlink(config_path)
    
    def test_config_values(self):
        """Test specific configuration values"""
        sim = IPSimulation()
        
        # Test physics dt
        self.assertAlmostEqual(sim.config['simulation']['physics_dt'], 1.0/60.0, places=6)
        
        # Test ground plane
        self.assertTrue(sim.config['scene']['ground_plane'])
        
        # Test equipment position
        self.assertEqual(len(sim.config['equipment']['position']), 3)
        
        # Test equipment orientation (quaternion)
        self.assertEqual(len(sim.config['equipment']['orientation']), 4)


class TestSimulationMethods(unittest.TestCase):
    """Test simulation methods without Isaac Sim"""
    
    def test_initialize_without_isaac_sim(self):
        """Test initialization without Isaac Sim installed"""
        sim = IPSimulation()
        sim.initialize()
        # Should not crash, just print warning
        # In mock mode, is_initialized will be False
        self.assertFalse(sim.is_initialized)
    
    def test_setup_scene_without_initialization(self):
        """Test setup_scene without initialization"""
        sim = IPSimulation()
        # Should not crash, just print warning
        sim.setup_scene()
    
    def test_add_equipment_without_initialization(self):
        """Test add_equipment without initialization"""
        sim = IPSimulation()
        # Should not crash, just print warning
        sim.add_equipment()
    
    def test_cleanup(self):
        """Test cleanup method"""
        sim = IPSimulation()
        # Should not crash even without initialization
        sim.cleanup()


if __name__ == '__main__':
    unittest.main()
