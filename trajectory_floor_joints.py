"""
IK-based trajectory generation for floor-mounted robot.
Uses proper Cartesian path geometries from trajectory.py and solves inverse kinematics
to generate accurate joint angles that produce correct welding shapes.

Based on paper: "Kinematics Analysis of 6 DOF Industrial Manipulator..."
- Case 1: Triangle (243mm base, 172mm sides)
- Case 2: Square (120×120mm)
- Case 3: Line (243mm) + Semicircle (radius 121mm)
"""

import numpy as np
from trajectory import generate_trajectory
from kinematics import RoboticArm6DOF

def generate_joint_trajectory(case_id):
    """
    Generates joint-space trajectories using IK from Cartesian waypoints.
    Robot base at Z=430mm (from DH parameters), reaching to work surface at Z=300mm.
    
    Args:
        case_id: 1 (Triangle), 2 (Square), or 3 (Line+Semicircle)
        
    Returns:
        List of frames with 'joints' (6 angles in degrees)
    """
    # Initialize robot with base at natural DH height (d1=430mm)
    robot = RoboticArm6DOF(base_height=430)
    
    # Generate Cartesian waypoints using proper geometry
    cartesian_points = generate_trajectory(case_id)
    
    frames = []
    ik_failures = 0
    last_good_joints = None
    
    print(f"\n=== Generating trajectory for Case {case_id} ===")
    print(f"Total Cartesian waypoints: {len(cartesian_points)}")
    
    # Debug: Print first few waypoints
    if len(cartesian_points) > 0:
        print(f"First waypoint: pos={cartesian_points[0]['pos']}, orient={cartesian_points[0]['orient']}")
        if len(cartesian_points) > 10:
            print(f"10th waypoint: pos={cartesian_points[10]['pos']}, orient={cartesian_points[10]['orient']}")
    
    for i, point in enumerate(cartesian_points):
        target_pos_world = point['pos']  # World coordinates
        target_orient = point['orient']
        
        # CRITICAL: Convert world coordinates to base-relative coordinates
        # Base is at [0, 0, 430], so subtract base height from Z
        target_pos_base = [
            target_pos_world[0],
            target_pos_world[1],
            target_pos_world[2] - 430  # Convert Z from world to base-relative
        ]
        
        try:
            # Solve inverse kinematics (expects base-relative coordinates)
            joints = robot.inverse_kinematics(target_pos_base, target_orient)
            
            # Validate solution with forward kinematics (returns world coordinates)
            T, origins = robot.forward_kinematics(joints, base_offset=[0, 0, 430])
            actual_pos_world = origins[-1]
            
            # Calculate position error (compare in world coordinates)
            error = np.linalg.norm(np.array(actual_pos_world) - np.array(target_pos_world))
            
            # Check if solution is acceptable (within 10mm tolerance)
            if error > 10:
                print(f"  [FAIL] Point {i}: IK FAILED! Target(world)={target_pos_world}, Target(base)={target_pos_base}, Error={error:.1f}mm")
                print(f"         Using fallback position")
                if last_good_joints is not None:
                    joints = last_good_joints
                    ik_failures += 1
                else:
                    # First point failed, use safe default
                    joints = [0, 20, -30, 0, 30, 0]
                    ik_failures += 1
            else:
                last_good_joints = joints.copy()
                
                # Log progress periodically
                if i % 20 == 0:
                    print(f"  [OK] Point {i}: Target(world)={target_pos_world}, Error={error:.2f}mm, "
                          f"Joints=[{joints[0]:.1f}, {joints[1]:.1f}, {joints[2]:.1f}, "
                          f"{joints[3]:.1f}, {joints[4]:.1f}, {joints[5]:.1f}]")
            
            # Check workspace constraints (end-effector should be above ground)
            if actual_pos_world[2] < 0:
                print(f"  WARNING: Point {i} underground (Z={actual_pos_world[2]:.1f}mm)")
            
            frames.append({'joints': joints})
            
        except Exception as e:
            print(f"  Point {i}: IK failed - {str(e)}")
            ik_failures += 1
            
            # Use last good solution or safe default
            if last_good_joints is not None:
                frames.append({'joints': last_good_joints})
            else:
                frames.append({'joints': [0, 20, -30, 0, 30, 0]})
    
    print(f"=== Trajectory generation complete ===")
    print(f"Total frames: {len(frames)}")
    print(f"IK failures: {ik_failures}")
    
    # Verify closure for closed shapes (Cases 1 and 2)
    if case_id in [1, 2] and len(frames) > 1:
        first_frame = frames[0]
        last_frame = frames[-1]
        
        # Check joint space closure
        joint_diff = np.array(first_frame['joints']) - np.array(last_frame['joints'])
        joint_closure_error = np.linalg.norm(joint_diff)
        
        # Check Cartesian closure
        _, origins_first = robot.forward_kinematics(first_frame['joints'], base_offset=[0, 0, 430])
        _, origins_last = robot.forward_kinematics(last_frame['joints'], base_offset=[0, 0, 430])
        cartesian_closure_error = np.linalg.norm(
            np.array(origins_first[-1]) - np.array(origins_last[-1])
        )
        
        print(f"Closure check:")
        print(f"  Joint space error: {joint_closure_error:.2f}°")
        print(f"  Cartesian closure error: {cartesian_closure_error:.2f}mm")
        
        if cartesian_closure_error > 20:
            print(f"  WARNING: Poor closure! Shape may not be closed properly.")
    
    return frames
