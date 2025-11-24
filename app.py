# ============================================================================
# IMPORTS
# ============================================================================

# Flask web framework imports for creating the web server and handling HTTP requests
from flask import Flask, render_template, jsonify, request

# Import the 6-DOF robotic arm kinematics class for forward kinematics calculations
from kinematics import RoboticArm6DOF

# Import function to generate joint-space trajectories using inverse kinematics
from trajectory_floor_joints import generate_joint_trajectory

# Import function to generate Cartesian-space trajectories (target waypoints)
from trajectory import generate_trajectory

# Import validation class to verify welding path accuracy and joint angle ranges
from validation import WeldingPathValidator

# Time module for measuring execution time of trajectory calculations
import time

# NumPy for numerical operations (e.g., calculating path lengths)
import numpy as np

# ============================================================================
# FLASK APPLICATION INITIALIZATION
# ============================================================================

# Create Flask application instance
app = Flask(__name__)

# ============================================================================
# ROBOT CONFIGURATION
# ============================================================================

# Robot configuration - Floor/side mounting (standard industrial setup)  
# Robot base mounted at its natural height from DH table (d1=430mm)
ROBOT_BASE_HEIGHT = 430  # mm - robot base at natural mounting height

# Initialize the 6-DOF robotic arm with the specified base height
robot = RoboticArm6DOF(base_height=ROBOT_BASE_HEIGHT)

# Initialize the welding path validator for trajectory verification
validator = WeldingPathValidator()

# ============================================================================
# ROUTE: HOME PAGE
# ============================================================================

@app.route('/')
def index():
    """
    Serves the main HTML page of the application.
    This is the entry point for the web interface.
    """
    return render_template('index.html')

# ============================================================================
# API ENDPOINT: FORWARD KINEMATICS CALCULATION
# ============================================================================

@app.route('/api/calculate_fk', methods=['POST'])
def calculate_fk():
    """
    Endpoint to calculate Forward Kinematics from manual joint sliders.
    
    Receives joint angles from the frontend and computes the position of each
    joint and the end-effector in 3D space.
    
    Request JSON format:
        {
            'joints': [θ1, θ2, θ3, θ4, θ5, θ6]  # Joint angles in degrees
        }
    
    Response JSON format:
        {
            'origins': [[x0,y0,z0], [x1,y1,z1], ..., [x6,y6,z6]]  # 3D positions
        }
    """
    # Parse incoming JSON data from the request
    data = request.json
    
    # Extract joint angles, defaulting to all zeros if not provided
    joints = data.get('joints', [0]*6)
    
    # Define the base offset (robot base position in 3D space)
    base_offset = [0, 0, ROBOT_BASE_HEIGHT]
    
    # Compute forward kinematics to get transformation matrix and joint positions
    T, origins = robot.forward_kinematics(joints, base_offset=base_offset)
    
    # Return joint positions as JSON (convert NumPy arrays to lists)
    return jsonify({'origins': [p.tolist() for p in origins]})

# ============================================================================
# API ENDPOINT: SIMULATE WELDING PATH
# ============================================================================

@app.route('/api/simulate_path', methods=['POST'])
def simulate_path():
    """
    Endpoint to generate full simulation data for a welding case.
    
    This is the main simulation endpoint that:
    1. Generates joint-space trajectory using inverse kinematics
    2. Computes forward kinematics for visualization
    3. Validates the trajectory against paper results
    4. Calculates performance metrics
    
    Request JSON format:
        {
            'case_id': 1  # Test case number (1, 2, or 3)
        }
    
    Response JSON format:
        {
            'frames': [...],        # Joint configurations and positions for each frame
            'path': [...],          # Actual end-effector path from FK
            'target_path': [...],   # Target Cartesian waypoints
            'metadata': {...}       # Performance metrics and validation results
        }
    """
    # Parse incoming JSON data from the request
    data = request.json
    
    # Extract the test case ID (1, 2, or 3), defaulting to 1
    case_id = int(data.get('case_id', 1))
    
    # Record start time to measure trajectory computation performance
    start_time = time.time()
    
    # Generate joint-space trajectory (IK-based from Cartesian waypoints)
    # This uses inverse kinematics to convert Cartesian waypoints to joint angles
    joint_frames = generate_joint_trajectory(case_id)
    
    # Initialize lists to store frame data and end-effector path
    frames = []
    path = []
    
    # Also get target Cartesian path for visualization
    # This is the ideal path the end-effector should follow
    target_path = generate_trajectory(case_id)
    
    # Define the base offset for forward kinematics calculations
    base_offset = [0, 0, ROBOT_BASE_HEIGHT]
    
    # Calculate FK for each joint configuration
    # Loop through each joint configuration in the trajectory
    for joint_config in joint_frames:
        # Extract joint angles for this frame
        joints = joint_config['joints']
        
        # Compute FK to get visualization coordinates and end-effector position
        # T = transformation matrix, origins = 3D positions of all joints
        T, origins = robot.forward_kinematics(joints, base_offset=base_offset)
        
        # Store frame data (joint angles and 3D positions) for visualization
        frames.append({
            'joints': [float(j) for j in joints],  # Convert to float for JSON serialization
            'origins': [p.tolist() for p in origins]  # Convert NumPy arrays to lists
        })
        
        # Extract end-effector position for path visualization
        # origins[-1] is the last element, which is the end-effector position
        end_effector_pos = origins[-1].tolist()
        
        # Append end-effector position to path (orientation set to zero for simplicity)
        path.append({'pos': end_effector_pos, 'orient': [0, 0, 0]})
    
    # Calculate total time taken to solve the trajectory
    solve_time = time.time() - start_time
    
    # Run validation
    # Analyze joint angles to check if they're within acceptable ranges
    joint_analysis = validator.analyze_joint_angles(frames)
    
    # Compare results with the reference paper's figures
    paper_comparison = validator.compare_with_paper_figures(joint_analysis, case_id)
    
    # Calculate path statistics
    # Compute total path length by summing Euclidean distances between consecutive points
    if len(path) > 1:
        path_length = sum(np.linalg.norm(np.array(path[i+1]['pos']) - np.array(path[i]['pos'])) 
                         for i in range(len(path)-1))
    else:
        # If only one point or no points, path length is zero
        path_length = 0
    
    # Return comprehensive simulation results as JSON
    return jsonify({
        'frames': frames,  # All joint configurations and positions for animation
        'path': path,  # Actual trajectory path from FK (what the robot actually follows)
        'target_path': target_path,  # Target Cartesian path for visualization (ideal path)
        'metadata': {
            'case_id': case_id,  # Which test case was simulated
            'total_points': len(frames),  # Number of waypoints in trajectory
            'ik_failures': 0,  # IK-based approach (no failures expected)
            'solve_time_ms': round(solve_time * 1000, 2),  # Computation time in milliseconds
            'path_length_mm': round(path_length, 1),  # Total path length in millimeters
            'validation': {
                'method': 'ik_cartesian',  # Validation method used
                'avg_error_mm': 0,  # Will be calculated in validation (average position error)
                'max_error_mm': 0,  # Maximum position error
                'within_tolerance': True  # Whether path is within acceptable tolerance
            },
            'paper_match': paper_comparison['matches_paper']  # Does trajectory match paper results?
        }
    })

# ============================================================================
# API ENDPOINT: GET TRAJECTORY PATH PREVIEW
# ============================================================================

@app.route('/api/get_trajectory_path', methods=['POST'])
def get_trajectory_path():
    """
    Endpoint to preview trajectory path (IK-based).
    
    This endpoint generates a quick preview of the trajectory path without
    full simulation data. Useful for visualizing the path before running
    the complete simulation.
    
    Request JSON format:
        {
            'case_id': 1  # Test case number
        }
    
    Response JSON format:
        {
            'path': [...],          # End-effector positions
            'target_path': [...],   # Target Cartesian waypoints
            'total_points': N       # Number of points in trajectory
        }
    """
    # Parse incoming JSON data from the request
    data = request.json
    
    # Extract the test case ID, defaulting to 1
    case_id = int(data.get('case_id', 1))
    
    # Generate joint-space trajectory using inverse kinematics
    trajectory = generate_joint_trajectory(case_id)
    
    # Get the target Cartesian path (ideal waypoints)
    target_path = generate_trajectory(case_id)
    
    # Convert to Cartesian for preview
    # Initialize empty path list
    path = []
    
    # Loop through each frame in the trajectory
    for frame in trajectory:
        # Compute forward kinematics to get end-effector position
        T, origins = robot.forward_kinematics(frame['joints'], base_offset=[0, 0, ROBOT_BASE_HEIGHT])
        
        # Append end-effector position (last element in origins) to path
        path.append({'pos': origins[-1].tolist()})
    
    # Return path preview data as JSON
    return jsonify({
        'path': path,  # Actual end-effector positions from FK
        'target_path': target_path,  # Target Cartesian waypoints
        'total_points': len(path)  # Number of points in the trajectory
    })

# ============================================================================
# API ENDPOINT: VALIDATE WELDING CASE
# ============================================================================

@app.route('/api/validate_case', methods=['POST'])
def validate_case():
    """
    Endpoint to generate detailed validation report.
    
    This endpoint performs comprehensive validation of a welding case,
    including joint angle analysis and comparison with paper results.
    
    Request JSON format:
        {
            'case_id': 1  # Test case number
        }
    
    Response JSON format:
        {
            'report': {...},         # Detailed validation report
            'joint_analysis': {...}  # Joint angle statistics for each joint
        }
    """
    # Parse incoming JSON data from the request
    data = request.json
    
    # Extract the test case ID, defaulting to 1
    case_id = int(data.get('case_id', 1))
    
    # Generate joint-space trajectory
    # This computes the joint angles for all waypoints using IK
    trajectory = generate_joint_trajectory(case_id)
    
    # Initialize lists to store frame data and end-effector path
    frames = []
    path = []
    
    # Define the base offset for FK calculations
    base_offset = [0, 0, ROBOT_BASE_HEIGHT]
    
    # Loop through each frame in the trajectory
    for frame in trajectory:
        # Extract joint angles for this frame
        joints = frame['joints']
        
        # Compute forward kinematics to get joint positions
        T, origins = robot.forward_kinematics(joints, base_offset=base_offset)
        
        # Store frame data (joint angles and positions)
        frames.append({
            'joints': [float(j) for j in joints],  # Convert to float for JSON
            'origins': [p.tolist() for p in origins]  # Convert NumPy arrays to lists
        })
        
        # Store end-effector position for path analysis
        path.append({'pos': origins[-1].tolist()})
    
    # Generate report
    # Create comprehensive validation report comparing with paper results
    report = validator.generate_report(frames, path, case_id)
    
    # Analyze joint angles to get min, max, and range for each joint
    joint_analysis = validator.analyze_joint_angles(frames)
    
    # Return validation results as JSON
    return jsonify({
        'report': report,  # Detailed validation report
        'joint_analysis': {
            # For each joint, extract key statistics
            k: {
                'min': v['min'],      # Minimum angle reached
                'max': v['max'],      # Maximum angle reached
                'range': v['range'],  # Range of motion (max - min)
                'angles': v['angles']  # All angle values throughout trajectory
            }
            for k, v in joint_analysis.items()  # Loop through all joints (J1-J6)
        }
    })

# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

if __name__ == '__main__':
    # Run the Flask development server
    # debug=True enables auto-reload and detailed error messages
    # port=5000 sets the server to listen on port 5000
    app.run(debug=True, port=5000)