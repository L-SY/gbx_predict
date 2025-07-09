#!/usr/bin/env python3
"""
Convert .npy homography matrix to YAML format for OpenCV loading
"""
import numpy as np
import yaml
import argparse
import os

def convert_npy_to_yaml(npy_path, yaml_path=None):
    """
    Convert .npy homography matrix file to YAML format
    
    Args:
        npy_path: Path to .npy file
        yaml_path: Output YAML path (optional, defaults to same name with .yaml extension)
    """
    if not os.path.exists(npy_path):
        print(f"Error: {npy_path} does not exist")
        return False
    
    try:
        # Load numpy array
        H = np.load(npy_path)
        print(f"Loaded homography matrix from {npy_path}")
        print(f"Matrix shape: {H.shape}")
        print(f"Matrix:\n{H}")
        
        # Validate matrix
        if H.shape != (3, 3):
            print(f"Error: Expected 3x3 matrix, got {H.shape}")
            return False
        
        # Determine output path
        if yaml_path is None:
            yaml_path = npy_path.replace('.npy', '.yaml')
        
        # Create OpenCV compatible YAML data structure
        yaml_data = {
            'homography': {
                'rows': 3,
                'cols': 3,
                'dt': 'd',  # double precision
                'data': H.flatten().tolist()
            }
        }
        
        # Also create a simple matrix format for easier loading
        simple_yaml_data = {
            'homography_matrix': H.tolist()
        }
        
        # Write YAML file with OpenCV format
        with open(yaml_path, 'w') as f:
            f.write("%YAML:1.0\n")
            f.write("---\n")
            f.write("homography: !!opencv-matrix\n")
            f.write("   rows: 3\n")
            f.write("   cols: 3\n")
            f.write("   dt: d\n")
            f.write("   data: [ ")
            data_str = ", ".join([f"{x:.16e}" for x in H.flatten()])
            f.write(data_str)
            f.write(" ]\n")
        
        print(f"Successfully converted to {yaml_path}")
        return True
        
    except Exception as e:
        print(f"Error during conversion: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Convert .npy homography matrix to YAML format')
    parser.add_argument('input', help='Input .npy file path')
    parser.add_argument('-o', '--output', help='Output YAML file path (optional)')
    
    args = parser.parse_args()
    
    success = convert_npy_to_yaml(args.input, args.output)
    if not success:
        exit(1)

if __name__ == '__main__':
    main() 