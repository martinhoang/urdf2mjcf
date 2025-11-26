"""
Tests for utility functions in _utils.py
"""
import pytest
import sys
import os

# Add parent directory to path to import _utils
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from _utils import parse_actuator_gains


class TestParseActuatorGains:
    """Test suite for parse_actuator_gains function"""
    
    def test_comma_separated_format(self):
        """Test comma-separated key=value pairs"""
        result = parse_actuator_gains('kp=500.0,kv=1.0')
        assert result == {'kp': 500.0, 'kv': 1.0}
    
    def test_space_separated_format(self):
        """Test space-separated key=value pairs"""
        result = parse_actuator_gains('kp=500.0 kv=1.0')
        assert result == {'kp': 500.0, 'kv': 1.0}
    
    def test_with_dampratio(self):
        """Test format with dampratio parameter"""
        result = parse_actuator_gains('kp=500.0,kv=1.0,dampratio=0.5')
        assert result == {'kp': 500.0, 'kv': 1.0, 'dampratio': 0.5}
    
    def test_dict_input(self):
        """Test that dictionary input passes through unchanged"""
        input_dict = {'kp': 500.0, 'kv': 1.0}
        result = parse_actuator_gains(input_dict)
        assert result == input_dict
        assert result is input_dict  # Should be same object
    
    def test_legacy_list_format(self):
        """Test legacy [kp, kv] list format"""
        result = parse_actuator_gains([500.0, 1.0])
        assert result == {'kp': 500.0, 'kv': 1.0}
    
    def test_legacy_list_integer_values(self):
        """Test legacy list format with integer values"""
        result = parse_actuator_gains([500, 1])
        assert result == {'kp': 500.0, 'kv': 1.0}
    
    def test_single_parameter(self):
        """Test single parameter format"""
        result = parse_actuator_gains('kp=500.0')
        assert result == {'kp': 500.0}
    
    def test_whitespace_handling(self):
        """Test that whitespace is properly handled"""
        result = parse_actuator_gains('kp = 500.0 , kv = 1.0')
        assert result == {'kp': 500.0, 'kv': 1.0}
    
    def test_all_three_parameters(self):
        """Test all three supported parameters"""
        result = parse_actuator_gains('kp=1000.0 kv=10.0 dampratio=0.7')
        assert result == {'kp': 1000.0, 'kv': 10.0, 'dampratio': 0.7}
    
    def test_scientific_notation(self):
        """Test scientific notation in values"""
        result = parse_actuator_gains('kp=5e2,kv=1e0')
        assert result == {'kp': 500.0, 'kv': 1.0}
    
    def test_negative_values(self):
        """Test negative values (should work even if not typical)"""
        result = parse_actuator_gains('kp=-500.0,kv=-1.0')
        assert result == {'kp': -500.0, 'kv': -1.0}
    
    # Error cases
    
    def test_invalid_key(self):
        """Test that invalid keys raise ValueError"""
        with pytest.raises(ValueError, match="Unknown actuator gain key: 'invalid'"):
            parse_actuator_gains('invalid=500.0,kv=1.0')
    
    def test_invalid_value(self):
        """Test that non-numeric values raise ValueError"""
        with pytest.raises(ValueError, match="Invalid value for 'kp': 'not_a_number'"):
            parse_actuator_gains('kp=not_a_number,kv=1.0')
    
    def test_missing_equals(self):
        """Test that missing equals sign raises ValueError"""
        with pytest.raises(ValueError, match="Invalid format"):
            parse_actuator_gains('kp500.0,kv1.0')
    
    def test_empty_string(self):
        """Test that empty string raises ValueError"""
        with pytest.raises(ValueError, match="No valid key=value pairs found"):
            parse_actuator_gains('')
    
    def test_legacy_list_wrong_length(self):
        """Test that list with wrong length raises ValueError"""
        with pytest.raises(ValueError, match="Legacy list format must have exactly 2 values"):
            parse_actuator_gains([500.0])
        
        with pytest.raises(ValueError, match="Legacy list format must have exactly 2 values"):
            parse_actuator_gains([500.0, 1.0, 0.5])
    
    def test_only_whitespace(self):
        """Test that only whitespace raises ValueError"""
        with pytest.raises(ValueError, match="No valid key=value pairs found"):
            parse_actuator_gains('   ')
    
    def test_duplicate_keys_last_wins(self):
        """Test that when keys are duplicated, the last value wins"""
        result = parse_actuator_gains('kp=500.0,kv=1.0,kp=1000.0')
        assert result['kp'] == 1000.0
        assert result['kv'] == 1.0


if __name__ == '__main__':
    # Run tests if executed directly
    pytest.main([__file__, '-v'])
