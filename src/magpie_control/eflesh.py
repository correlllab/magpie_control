#!/usr/bin/env python

import time
import numpy as np
from anyskin import AnySkinProcess


class EfleshDriver:
    """
    Thin wrapper for eflesh tactile sensors providing calibrated force readings.
    
    Force calibration (based on norm of sensor_data - baseline):
    - norm ~140: 0N
    - norm ~180: 0.1N
    - norm ~350: 5N
    - norm ~750: 10N
    """
    
    def __init__(self, port='/dev/ttyACM0', num_mags=5, auto_start=True):
        """
        Initialize eflesh sensor.
        
        Args:
            port: Serial port for the sensor (e.g., '/dev/ttyACM0')
            num_mags: Number of magnetometer sensors (default 5)
            auto_start: If True, automatically start the sensor stream
        """
        self.port = port
        self.num_mags = num_mags
        self.baseline = None
        self.sensor_stream = None
        self.is_streaming = False
        
        # Force calibration points (norm, force_N)
        self.calibration_points = np.array([
            [140, 0.0],
            [180, 0.1],
            [350, 5.0],
            [750, 10.0]
        ])
        
        if auto_start:
            self.start()
    
    def start(self):
        """Start the sensor stream and initialize baseline."""
        if not self.is_streaming:
            self.sensor_stream = AnySkinProcess(
                num_mags=self.num_mags,
                port=self.port,
            )
            self.sensor_stream.start()
            time.sleep(1.0)  # Allow sensor to stabilize
            self.is_streaming = True
            self.zero()  # Get initial baseline
    
    def stop(self):
        """Stop the sensor stream."""
        if self.is_streaming and self.sensor_stream is not None:
            self.sensor_stream.pause_streaming()
            self.sensor_stream.join()
            self.is_streaming = False
    
    def zero(self, num_samples=5):
        """
        Zero the sensor by capturing current readings as baseline.
        
        Args:
            num_samples: Number of samples to average for baseline
        
        Returns:
            baseline: The baseline values (15D array for 5 magnetometers)
        """
        if not self.is_streaming:
            raise RuntimeError("Sensor stream not started. Call start() first.")
        
        baseline_data = self.sensor_stream.get_data(num_samples=num_samples)
        baseline_data = np.array(baseline_data)[:, 1:]  # Remove timestamp
        self.baseline = np.mean(baseline_data, axis=0)
        return self.baseline
    
    def get_raw_data(self, num_samples=1):
        """
        Get raw sensor data.
        
        Args:
            num_samples: Number of samples to read
        
        Returns:
            data: Raw sensor data array of shape (num_samples, 15)
        """
        if not self.is_streaming:
            raise RuntimeError("Sensor stream not started. Call start() first.")
        
        data = self.sensor_stream.get_data(num_samples=num_samples)
        data = np.array(data)[:, 1:]  # Remove timestamp
        return data
    
    def get_baseline_corrected_data(self, num_samples=1):
        """
        Get sensor data with baseline correction applied.
        
        Args:
            num_samples: Number of samples to read
        
        Returns:
            corrected_data: Baseline-corrected sensor data
        """
        if self.baseline is None:
            raise RuntimeError("Baseline not set. Call zero() first.")
        
        raw_data = self.get_raw_data(num_samples=num_samples)
        return raw_data - self.baseline
    
    def get_norm(self, num_samples=1):
        """
        Get the L2 norm of baseline-corrected sensor data.
        
        Args:
            num_samples: Number of samples to read and average
        
        Returns:
            norm: L2 norm of the baseline-corrected data
        """
        corrected_data = self.get_baseline_corrected_data(num_samples=num_samples)
        # Average across samples if multiple samples taken
        if num_samples > 1:
            corrected_data = np.mean(corrected_data, axis=0)
        return np.linalg.norm(corrected_data)
    
    def norm_to_force(self, norm):
        """
        Convert sensor norm to force in Newtons using piecewise linear interpolation.
        
        Args:
            norm: L2 norm of baseline-corrected sensor data
        
        Returns:
            force: Estimated force in Newtons
        """
        # Use piecewise linear interpolation between calibration points
        if norm <= self.calibration_points[0, 0]:
            # Below minimum calibration point
            return 0.0
        elif norm >= self.calibration_points[-1, 0]:
            # Above maximum calibration point - extrapolate linearly
            slope = (self.calibration_points[-1, 1] - self.calibration_points[-2, 1]) / \
                    (self.calibration_points[-1, 0] - self.calibration_points[-2, 0])
            return self.calibration_points[-1, 1] + slope * (norm - self.calibration_points[-1, 0])
        else:
            # Interpolate between calibration points
            return np.interp(norm, self.calibration_points[:, 0], self.calibration_points[:, 1])
    
    def get_force(self, num_samples=1):
        """
        Get force reading in Newtons.
        
        Args:
            num_samples: Number of samples to read and average
        
        Returns:
            force: Estimated force in Newtons
        """
        norm = self.get_norm(num_samples=num_samples)
        return self.norm_to_force(norm)
    
    def get_force_per_mag(self, num_samples=1):
        """
        Get force reading for each magnetometer individually.
        
        Args:
            num_samples: Number of samples to read and average
        
        Returns:
            forces: Array of 5 force values (one per magnetometer) in Newtons
        """
        corrected_data = self.get_baseline_corrected_data(num_samples=num_samples)
        if num_samples > 1:
            corrected_data = np.mean(corrected_data, axis=0)
        
        # Reshape to (5, 3) for 5 magnetometers with x,y,z
        corrected_data = corrected_data.reshape(5, 3)
        
        # Calculate norm for each magnetometer
        norms = np.linalg.norm(corrected_data, axis=1)
        
        # Convert each norm to force
        forces = np.array([self.norm_to_force(norm) for norm in norms])
        return forces
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
    
    def __del__(self):
        """Cleanup on deletion."""
        if self.is_streaming:
            self.stop()


if __name__ == "__main__":
    # Example usage
    import argparse
    
    parser = argparse.ArgumentParser(description="Test eflesh driver")
    parser.add_argument("-p", "--port", type=str, default="/dev/ttyACM0",
                        help="Serial port for eflesh sensor")
    args = parser.parse_args()
    
    print(f"Testing eflesh driver on port {args.port}")
    
    with EfleshDriver(port=args.port) as sensor:
        print("Sensor initialized and zeroed")
        print(f"Baseline: {sensor.baseline}")
        
        print("\nReading forces for 10 seconds...")
        start_time = time.time()
        while time.time() - start_time < 10:
            force = sensor.get_force(num_samples=3)
            norm = sensor.get_norm(num_samples=3)
            print(f"Norm: {norm:.2f}, Force: {force:.3f} N")
            time.sleep(0.1)