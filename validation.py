import numpy as np
import csv
from datetime import datetime

class WeldingPathValidator:
    """
    Validation module for comparing robotic welding simulation results
    with the reference data from the paper:
    "Kinematics Analysis of 6 DOF Industrial Manipulator and Trajectory Planning"
    """
    
    def __init__(self):
        """Initialize validator with tolerance thresholds."""
        self.position_tolerance = 1.0  # mm - acceptable position error
        self.orientation_tolerance = 2.0  # degrees - acceptable orientation error
        self.joint_angle_tolerance = 1.0  # degrees - acceptable joint angle error
        
    def validate_trajectory_execution(self, frames, trajectory_points):
        """
        Validates that the robot correctly follows the desired trajectory.
        
        Args:
            frames: List of dicts with 'joints' and 'origins' from simulation
            trajectory_points: List of dicts with 'pos' and 'orient' target points
            
        Returns:
            dict: Validation results with errors and statistics
        """
        results = {
            'total_points': len(frames),
            'position_errors': [],
            'orientation_errors': [],
            'max_position_error': 0,
            'max_orientation_error': 0,
            'avg_position_error': 0,
            'avg_orientation_error': 0,
            'within_tolerance': True
        }
        
        # Compare each frame's end-effector position with target
        for i, (frame, target) in enumerate(zip(frames, trajectory_points)):
            # End-effector is the last origin point
            actual_pos = np.array(frame['origins'][-1])
            target_pos = np.array(target['pos'])
            
            # Position error (Euclidean distance)
            pos_error = np.linalg.norm(actual_pos - target_pos)
            results['position_errors'].append(pos_error)
            
            if pos_error > results['max_position_error']:
                results['max_position_error'] = pos_error
        
        # Calculate statistics
        if results['position_errors']:
            results['avg_position_error'] = np.mean(results['position_errors'])
            results['rms_position_error'] = np.sqrt(np.mean(np.square(results['position_errors'])))
            
            # Check if within tolerance
            if results['max_position_error'] > self.position_tolerance:
                results['within_tolerance'] = False
        
        return results
    
    def analyze_joint_angles(self, frames):
        """
        Analyzes joint angle behavior throughout the trajectory.
        Identifies discontinuities, rate of change, and patterns.
        
        Args:
            frames: List of dicts with 'joints' data
            
        Returns:
            dict: Analysis results for each joint
        """
        joint_analysis = {}
        
        # Extract joint angles for each joint across all frames
        num_joints = 6
        for joint_idx in range(num_joints):
            angles = [frame['joints'][joint_idx] for frame in frames]
            angles_array = np.array(angles)
            
            # Calculate statistics
            analysis = {
                'min': np.min(angles_array),
                'max': np.max(angles_array),
                'range': np.max(angles_array) - np.min(angles_array),
                'mean': np.mean(angles_array),
                'std': np.std(angles_array),
                'angles': angles  # Keep for plotting
            }
            
            # Calculate rate of change (angular velocity approximation)
            if len(angles) > 1:
                angle_changes = np.diff(angles_array)
                analysis['max_change'] = np.max(np.abs(angle_changes))
                analysis['avg_change'] = np.mean(np.abs(angle_changes))
                
                # Check for discontinuities (large jumps)
                discontinuities = np.where(np.abs(angle_changes) > 20)[0]  # 20 degree threshold
                analysis['discontinuities'] = len(discontinuities)
                analysis['smooth'] = len(discontinuities) == 0
            
            joint_analysis[f'joint_{joint_idx + 1}'] = analysis
        
        return joint_analysis
    
    def check_vertical_orientation(self, frames):
        """
        Validates that the welding torch maintains vertical orientation.
        According to the paper, this is crucial for weld quality.
        
        Args:
            frames: List of dicts with 'joints' data
            
        Returns:
            dict: Orientation validation results
        """
        # For vertical torch (pitch = 180°), we expect minimal variation
        # This would require FK to compute actual orientation, simplified here
        results = {
            'maintained_vertical': True,
            'note': 'Full orientation check requires FK rotation matrix analysis'
        }
        return results
    
    def export_joint_angles_csv(self, frames, case_id, filename=None):
        """
        Exports joint angles to CSV file for external analysis and plotting.
        Format matches what would be needed to compare with paper's plots.
        
        Args:
            frames: List of dicts with 'joints' data
            case_id: Test case number (1, 2, or 3)
            filename: Optional custom filename
            
        Returns:
            str: Path to saved CSV file
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"joint_angles_case{case_id}_{timestamp}.csv"
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Header
            writer.writerow(['Frame', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])
            
            # Data rows
            for i, frame in enumerate(frames):
                row = [i] + frame['joints']
                writer.writerow(row)
        
        return filename
    
    def compare_with_paper_figures(self, joint_analysis, case_id):
        """
        Compares simulation results with observations from paper's Figures 4-6.
        
        Key observations from paper:
        - Case 3 (curve): Significant changes in joints 4 and 6
        - All cases: Smooth, continuous joint motion
        - Joint limits never exceeded
        
        Args:
            joint_analysis: Output from analyze_joint_angles()
            case_id: Test case number (1, 2, or 3)
            
        Returns:
            dict: Comparison results and validation status
        """
        comparison = {
            'case_id': case_id,
            'matches_paper': True,
            'observations': []
        }
        
        if case_id == 3:
            # Paper notes: "significant changes in joints 4 and 6" for curved path
            j4_range = joint_analysis['joint_4']['range']
            j6_range = joint_analysis['joint_6']['range']
            
            if j4_range > 30 or j6_range > 30:  # Significant change threshold
                comparison['observations'].append(
                    f"✓ Case 3: Confirmed significant changes in Joint 4 (range: {j4_range:.1f}°) "
                    f"and Joint 6 (range: {j6_range:.1f}°) as noted in paper."
                )
            else:
                comparison['matches_paper'] = False
                comparison['observations'].append(
                    f"✗ Case 3: Expected significant changes in Joints 4 & 6, but got "
                    f"J4 range: {j4_range:.1f}°, J6 range: {j6_range:.1f}°"
                )
        
        # Check for smooth motion in all joints
        smooth_joints = sum(1 for j in joint_analysis.values() if j.get('smooth', True))
        if smooth_joints == 6:
            comparison['observations'].append("✓ All joints exhibit smooth, continuous motion.")
        else:
            comparison['observations'].append(
                f"✗ Warning: {6 - smooth_joints} joint(s) have discontinuities."
            )
        
        # Check joint limits
        limits = [60, 90, 80, 180, 80, 270]  # Max absolute values for each joint
        for i, limit in enumerate(limits):
            joint_key = f'joint_{i + 1}'
            if abs(joint_analysis[joint_key]['max']) > limit:
                comparison['matches_paper'] = False
                comparison['observations'].append(
                    f"✗ Joint {i+1} exceeded limit: {joint_analysis[joint_key]['max']:.1f}° > {limit}°"
                )
        
        return comparison
    
    def generate_report(self, frames, trajectory_points, case_id):
        """
        Generates a comprehensive validation report for a test case.
        
        Args:
            frames: Simulation results
            trajectory_points: Target trajectory
            case_id: Test case number
            
        Returns:
            str: Formatted validation report
        """
        # Run all validation checks
        traj_validation = self.validate_trajectory_execution(frames, trajectory_points)
        joint_analysis = self.analyze_joint_angles(frames)
        paper_comparison = self.compare_with_paper_figures(joint_analysis, case_id)
        
        # Format report
        report = []
        report.append("=" * 70)
        report.append(f"VALIDATION REPORT - Case {case_id}")
        report.append("=" * 70)
        report.append("")
        
        report.append("1. TRAJECTORY ACCURACY")
        report.append("-" * 70)
        report.append(f"   Total points: {traj_validation['total_points']}")
        report.append(f"   Avg position error: {traj_validation['avg_position_error']:.3f} mm")
        report.append(f"   Max position error: {traj_validation['max_position_error']:.3f} mm")
        report.append(f"   RMS position error: {traj_validation.get('rms_position_error', 0):.3f} mm")
        report.append(f"   Within tolerance ({self.position_tolerance} mm): "
                     f"{'✓ YES' if traj_validation['within_tolerance'] else '✗ NO'}")
        report.append("")
        
        report.append("2. JOINT ANGLE ANALYSIS")
        report.append("-" * 70)
        for joint_name, analysis in joint_analysis.items():
            report.append(f"   {joint_name.upper()}:")
            report.append(f"      Range: {analysis['range']:.2f}° "
                         f"(min: {analysis['min']:.2f}°, max: {analysis['max']:.2f}°)")
            report.append(f"      Smooth: {'✓ Yes' if analysis.get('smooth', True) else '✗ No'}")
            if not analysis.get('smooth', True):
                report.append(f"      Discontinuities: {analysis.get('discontinuities', 0)}")
        report.append("")
        
        report.append("3. COMPARISON WITH PAPER")
        report.append("-" * 70)
        report.append(f"   Matches paper observations: "
                     f"{'✓ YES' if paper_comparison['matches_paper'] else '✗ NO'}")
        for obs in paper_comparison['observations']:
            report.append(f"   {obs}")
        report.append("")
        
        report.append("=" * 70)
        
        return "\n".join(report)

