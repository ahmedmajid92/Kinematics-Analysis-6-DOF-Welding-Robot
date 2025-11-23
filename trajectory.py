import numpy as np

def generate_trajectory(case_id):
    """
    Generates Cartesian trajectory waypoints for welding cases from the paper.
    Robot base is at ground level (Z=0), work surface at Z=300mm.
    
    Based on paper "Kinematics Analysis of 6 DOF Industrial Manipulator...":
    - Case 1: Isosceles triangle (base 243mm, equal sides ~172mm)
    - Case 2: Square/Rectangle (120mm × 120mm)
    - Case 3: Straight line (243mm) + connected semicircle (radius 121mm)
    
    All maintain vertical torch orientation for proper welding.
    """
    points = []
    
    # Torch orientation - VERTICAL for floor welding
    # Robot base at Z=430mm, welding at Z=-100mm (below floor level)
    # Torch points straight down (negative Z direction): Roll=0°, Pitch=-90° (downward), Yaw=0°
    torch_orient = [0, -90, 0]  # Pointing straight down for welding
    
    # Workspace center - optimized for floor-mounted robot (base at Z=430mm)
    # Robot reaches forward and down to floor level for welding
    # Reachable workspace: X=400-900mm, Y=±300mm, Z=0-400mm
    center_x = 650  # mm forward from robot base (comfortable reach)
    center_y = 0    # mm (centered left-right)
    weld_z = -100   # mm height (below floor level)
    
    # Helper for Linear Interpolation
    def add_segment(start_pt, end_pt, count=30):
        """Add linearly interpolated points between start and end."""
        for i in range(count):
            t = i / (count - 1) if count > 1 else 0
            pos = start_pt + t * (end_pt - start_pt)
            points.append({'pos': pos.tolist(), 'orient': torch_orient})
    
    if case_id == 1:
        # Case 1: Isosceles Triangle from paper
        # Base: 243mm (horizontal), Equal sides: ~172mm each
        # Height: sqrt(172² - 121.5²) ≈ 122mm
        
        base_width = 243  # mm
        height = 122      # mm
        
        # Triangle vertices (centered at work area)
        p1 = np.array([center_x - base_width/2, center_y, weld_z])  # Left corner
        p2 = np.array([center_x + base_width/2, center_y, weld_z])  # Right corner
        p3 = np.array([center_x, center_y + height, weld_z])        # Top apex
        
        # Draw closed triangle: p1 -> p2 -> p3 -> p1
        add_segment(p1, p2, count=30)  # Base (horizontal)
        add_segment(p2, p3, count=25)  # Right side
        add_segment(p3, p1, count=25)  # Left side
        
    elif case_id == 2:
        # Case 2: Square from paper (120mm × 120mm)
        
        side_length = 120  # mm
        half_side = side_length / 2
        
        # Square vertices (centered at work area)
        p1 = np.array([center_x - half_side, center_y - half_side, weld_z])  # Bottom-left
        p2 = np.array([center_x + half_side, center_y - half_side, weld_z])  # Bottom-right
        p3 = np.array([center_x + half_side, center_y + half_side, weld_z])  # Top-right
        p4 = np.array([center_x - half_side, center_y + half_side, weld_z])  # Top-left
        
        # Draw closed square: p1 -> p2 -> p3 -> p4 -> p1
        add_segment(p1, p2, count=25)  # Bottom side
        add_segment(p2, p3, count=25)  # Right side
        add_segment(p3, p4, count=25)  # Top side
        add_segment(p4, p1, count=25)  # Left side
        
    elif case_id == 3:
        # Case 3: Composite Path - Straight Line + Connected Semicircle
        # Line: 243mm, Semicircle radius: 121.5mm (forms "D" shape)
        # KEY: Shows significant J4 and J6 rotation (paper observation)
        
        line_length = 243  # mm
        radius = line_length / 2  # 121.5mm (exact half for perfect closure)
        
        # Part 1: Straight line (vertical left edge of "D")
        # Line positioned at center
        line_start = np.array([center_x, center_y - radius, weld_z])
        line_end = np.array([center_x, center_y + radius, weld_z])
        add_segment(line_start, line_end, count=40)
        
        # Part 2: Semicircle connecting back to start (right bulge of "D")
        # Arc from top of line, curving RIGHT, back to bottom
        # Center at same position as line (creates proper D shape)
        arc_center = np.array([center_x, center_y, weld_z])
        
        # Generate semicircular arc from top to bottom, going clockwise (right side)
        # Arc from 90° (top) through 0° (max right) to -90° (bottom)
        num_arc_points = 50
        angle_start = np.pi / 2      # 90° - top point (center_x, center_y + radius)
        angle_end = -np.pi / 2       # -90° - bottom point (center_x, center_y - radius)
        
        for i in range(num_arc_points + 1):
            t = i / num_arc_points
            angle = angle_start + (angle_end - angle_start) * t  # Sweep from 90° to -90°
            
            x = arc_center[0] + radius * np.cos(angle)
            y = arc_center[1] + radius * np.sin(angle)
            z = weld_z
            
            points.append({
                'pos': [x, y, z],
                'orient': torch_orient
            })
    
    return points
