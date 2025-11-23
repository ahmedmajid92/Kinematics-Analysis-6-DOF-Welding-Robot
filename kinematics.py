import numpy as np

class RoboticArm6DOF:
    def __init__(self, base_height=0):
        """
        Initialize the 6-DOF Manipulator with DH parameters from Table 1 of the paper.
        Ref:  "Kinematics Analysis of 6 DOF Industrial Manipulator..."
        
        Args:
            base_height: Height at which robot base is mounted (mm). Default 0 (floor).
                        For overhead mounting (ceiling), use positive value like 800.
        
        DH Convention (Standard): Each transformation from frame i-1 to frame i:
        1. Rotate about z_{i-1} by theta_i
        2. Translate along z_{i-1} by d_i
        3. Translate along x_i by a_i
        4. Rotate about x_i by alpha_i
        """
        # Store base mounting height
        self.base_height = base_height
        
        # DH Parameters from paper's Table 1: [alpha (deg), a (mm), d (mm), theta_min (deg), theta_max (deg)]
        self.dh_params = [
            {'alpha': -90, 'a': 160, 'd': 430, 'min': -60, 'max': 60},  # Joint 1
            {'alpha': 180, 'a': 580, 'd': 0,   'min': 0,   'max': 90},  # Joint 2
            {'alpha': 90,  'a': 125, 'd': 0,   'min': -80, 'max': 80},  # Joint 3
            {'alpha': -90, 'a': 0,   'd': 239, 'min': -180,'max': 180}, # Joint 4
            {'alpha': 90,  'a': 0,   'd': 0,   'min': -80, 'max': 80},  # Joint 5
            {'alpha': 0,   'a': 0,   'd': 411, 'min': -270,'max': 270}  # Joint 6
        ]
        
        # Precompute conversion constants
        self.d2r = np.pi / 180.0
        self.r2d = 180.0 / np.pi
        
        # Robot link lengths for IK calculations
        self.d1 = self.dh_params[0]['d']  # 430 mm (base height)
        self.a1 = self.dh_params[0]['a']  # 160 mm (base offset)
        self.a2 = self.dh_params[1]['a']  # 580 mm (upper arm)
        self.a3 = self.dh_params[2]['a']  # 125 mm (forearm)
        self.d4 = self.dh_params[3]['d']  # 239 mm (wrist offset)
        self.d6 = self.dh_params[5]['d']  # 411 mm (tool length)

    def _dh_matrix(self, theta, alpha, a, d):
        """
        Computes the homogenous transformation matrix T_i^{i-1}
        Equation (1) in the research snippet.
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha * self.d2r)
        sa = np.sin(alpha * self.d2r)

        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joints, base_offset=None):
        """
        Calculates the position and orientation of the end-effector.
        Also returns intermediate frame origins for visualization.
        
        Args:
            joints: List of 6 joint angles in degrees
            base_offset: [x, y, z] offset for robot base mounting (default: uses self.base_height)
        """
        if base_offset is None:
            base_offset = np.array([0, 0, self.base_height])
        else:
            base_offset = np.array(base_offset)
            
        T = np.eye(4)
        origins = [base_offset.copy()] # Base is at specified offset

        for i in range(6):
            # Clamp joint input to limits defined in Table 1
            mn = self.dh_params[i]['min']
            mx = self.dh_params[i]['max']
            angle = max(mn, min(mx, joints[i]))
            
            theta = angle * self.d2r
            params = self.dh_params[i]
            
            # Compute transformation for this link
            T_i = self._dh_matrix(theta, params['alpha'], params['a'], params['d'])
            
            # Chain transformations: T_total = T_prev * T_curr
            T = T @ T_i
            
            # Store origin of the new frame (with base offset)
            origins.append(T[:3, 3] + base_offset)

        return T, origins

    def inverse_kinematics(self, target_pos, target_orient_rpy):
        """
        Solves the Inverse Kinematics problem using numerical optimization.
        This approach uses scipy.optimize to find joint angles that minimize
        the error between desired and actual end-effector pose.
        
        Args:
            target_pos: [x, y, z] in mm (end-effector position, BASE-RELATIVE coordinates)
            target_orient_rpy: [roll, pitch, yaw] in degrees (ZYX Euler angles)
            
        Returns:
            List of 6 joint angles in degrees.
        """
        from scipy.optimize import minimize
        
        # Calculate target rotation matrix
        roll, pitch, yaw = [ang * self.d2r for ang in target_orient_rpy]
        Rx = np.array([[1, 0, 0], 
                       [0, np.cos(roll), -np.sin(roll)], 
                       [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], 
                       [0, 1, 0], 
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], 
                       [np.sin(yaw), np.cos(yaw), 0], 
                       [0, 0, 1]])
        R_target = Rz @ Ry @ Rx
        
        # Target position in world coordinates (add base height)
        target_pos_world = np.array([target_pos[0], target_pos[1], target_pos[2] + self.base_height])
        
        def error_function(joints_deg):
            """
            Calculate error between desired and actual pose.
            Returns scalar error to minimize.
            """
            # Forward kinematics
            T, origins = self.forward_kinematics(joints_deg, base_offset=[0, 0, self.base_height])
            actual_pos = np.array(origins[-1])
            actual_rot = T[:3, :3]
            
            # Position error (mm)
            pos_error = np.linalg.norm(actual_pos - target_pos_world)
            
            # Orientation error (Frobenius norm of rotation matrix difference)
            # Scale down to make it comparable to position error
            rot_error = np.linalg.norm(actual_rot - R_target, 'fro') * 100
            
            # Total error (weighted sum)
            total_error = pos_error + 0.5 * rot_error
            
            return total_error
        
        # Initial guess - start from home position or use geometric approximation
        # Simple initial guess based on target position
        theta1_guess = np.arctan2(target_pos[1], target_pos[0]) * self.r2d
        theta2_guess = 30.0  # Reasonable default
        theta3_guess = -20.0  # Reasonable default
        
        initial_guess = [theta1_guess, theta2_guess, theta3_guess, 0, 0, 0]
        
        # Bounds for each joint (from DH parameters)
        bounds = [(self.dh_params[i]['min'], self.dh_params[i]['max']) for i in range(6)]
        
        # Optimize using L-BFGS-B method (handles bounds well)
        result = minimize(
            error_function,
            initial_guess,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 200, 'ftol': 1e-6}
        )
        
        if result.success or result.fun < 10:  # Accept if error < 10mm
            return result.x.tolist()
        else:
            # If optimization failed, try again with different initial guess
            initial_guess_alt = [0, 45, -30, 0, 0, 0]
            result = minimize(
                error_function,
                initial_guess_alt,
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': 200, 'ftol': 1e-6}
            )
            return result.x.tolist()