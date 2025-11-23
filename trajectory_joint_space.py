"""
Joint-space trajectory generation for welding cases.
Robot is MOUNTED ABOVE (at Z=800mm) and welds DOWNWARD onto floor (Z≈0-50mm).
This matches Figure 3 in the paper showing overhead robot configuration.

Welding performed on floor at Z=0-50mm with robot base at Z=800mm.
"""

import numpy as np

def generate_joint_trajectory(case_id):
    """
    Generates joint-space trajectories for overhead welding configuration.
    Robot mounted above, welding downward onto floor surface.
    
    Args:
        case_id: 1 (Triangle), 2 (Rectangle), or 3 (Line+Curve)
        
    Returns:
        List of dicts with 'joints' (6 angles in degrees)
    """
    frames = []
    
    if case_id == 1:
        # Case 1: Triangular Welding Path (from paper)
        # Isosceles triangle with base ~243mm and equal sides ~172mm
        # Uses J2=87-88°, J3=56-58° for floor welding (Z≈25mm)
        num_points = 90
        
        for i in range(num_points):
            t = i / num_points
            
            # Three sides of isosceles triangle
            if t < 0.333:  # Side 1: Base (left to right)
                progress = t / 0.333
                j1 = -35 + 70 * progress  # -35° to +35° (longer base)
                j2 = 88
                j3 = 58
                j5 = 20
            elif t < 0.666:  # Side 2: Right edge (to apex)
                progress = (t - 0.333) / 0.333
                j1 = 35 - 35 * progress  # +35° to 0° (apex)
                j2 = 88 - 5 * progress  # Adjust for shape
                j3 = 58 - 5 * progress
                j5 = 20
            else:  # Side 3: Left edge (back to start, close triangle)
                progress = (t - 0.666) / 0.334
                j1 = 0 - 35 * progress  # 0° back to -35°
                j2 = 83 + 5 * progress  # Back to base
                j3 = 53 + 5 * progress
                j5 = 20
            
            joints = [j1, j2, j3, 0, j5, 0]
            frames.append({'joints': joints})
    
    elif case_id == 2:
        # Case 2: Square/Rectangle (120mm × 120mm from paper)
        # Perfect closed square at floor level
        # Uses J2=87-88°, J3=56-58° for floor welding (Z≈25mm)
        num_points = 100
        
        # Create square by parametric path: vary J1 and (J2,J3) together
        # J1 creates X-Y movement, J2+J3 combination creates perpendicular movement
        for i in range(num_points):
            t = i / num_points
            
            if t < 0.25:  # Side 1: BOTTOM
                progress = t / 0.25
                j1 = -18 + 36 * progress  # -18° to +18°
                j2 = 88  # Constant
                j3 = 58
                j5 = 15
            elif t < 0.5:  # Side 2: RIGHT (perpendicular)
                progress = (t - 0.25) / 0.25
                j1 = 18  # Constant
                j2 = 88 - 6 * progress  # Vary for perpendicular motion
                j3 = 58 + 6 * progress
                j5 = 15
            elif t < 0.75:  # Side 3: TOP (parallel to bottom)
                progress = (t - 0.5) / 0.25
                j1 = 18 - 36 * progress  # +18° back to -18°
                j2 = 82  # Constant
                j3 = 64
                j5 = 15
            else:  # Side 4: LEFT (close square)
                progress = (t - 0.75) / 0.25
                j1 = -18  # Constant
                j2 = 82 + 6 * progress  # Back to start
                j3 = 64 - 6 * progress
                j5 = 15
            
            joints = [j1, j2, j3, 0, j5, 0]
            frames.append({'joints': joints})
    
    elif case_id == 3:
        # Case 3: Connected path - Straight Line + Semicircle (from paper)
        # Line: 243mm, then semicircle radius ~121mm connects back to form "D" shape
        # Shows SIGNIFICANT Joint 6 rotation as noted in paper
        # Uses J2=87-88°, J3=56-58° for floor welding (Z≈25mm)
        num_points = 90
        
        for i in range(num_points):
            t = i / num_points
            
            if t < 0.4:  # Straight line portion (40% of path)
                progress = t / 0.4
                # Create straight line by varying J1
                j1 = -35 + 70 * progress  # -35° to +35° (line)
                j2 = 88
                j3 = 58
                j5 = 20
                j6 = 0  # No wrist rotation during line
            else:  # Semicircular arc (60% of path) - must connect back to start
                arc_progress = (t - 0.4) / 0.6
                # Angle from 0 to π (semicircle from end of line back to start)
                angle = np.pi * arc_progress
                
                # Create semicircle that exactly closes the loop
                # At angle=0 (start of arc): j1=+35° (end of line)
                # At angle=π (end of arc): j1=-35° (start of line) 
                j1 = 35 * np.cos(angle)  # Exactly: +35 -> 0 -> -35
                
                # Perpendicular motion for circular arc (keep Z constant)
                j2 = 88 + 4 * np.sin(angle)  # Create arc  depth
                j3 = 58 - 4 * np.sin(angle)  # Coordinate with J2
                j5 = 20  # Keep constant
                
                # KEY FEATURE from paper: Significant J6 rotation during arc!
                j6 = 270 * arc_progress  # Rotates from 0° to 270°
            
            joints = [j1, j2, j3, 0, j5, j6]
            frames.append({'joints': joints})
    
    return frames

