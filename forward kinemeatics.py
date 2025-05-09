import numpy as np

end_effector_local = np.array([0, 0, 0, 1]) #x,y,z

def rotation_base(b_rad):  #rotate base joint
    cos, sin, = np.cos(b_rad), np.sin(b_rad)
    return np.array([
        [cos, -sin, 0],
        [sin, cos, 0], #standard rotation matrix
        [0, 0, 1]
    ])

def xyz_radians(radiansX, radiansY):
    cosX, sinX = np.cos(radiansX), np.sin(radiansX)
    cosY, sinY = np.cos(radiansY), np.sin(radiansY)

    rotationX = np.array([
        [1,  0,   0],
        [0, cosX, -sinX],
        [0, sinX,  cosX]
    ])

    rotationY = np.array([
        [ cosY, 0, sinY],
        [  0, 1,  0],
        [-sinY, 0, cosY]
    ])

    return rotationX @ rotationY

def transform(r_matrix, t_vector): #transform
    transform = np.eye(4)
    transform[:3, :3] = r_matrix
    transform[:3, 3] = t_vector
    return transform

import numpy as np

def rotation_base(b_rad):  # rotate base joint
    cos, sin = np.cos(b_rad), np.sin(b_rad)
    return np.array([
        [cos, -sin, 0],
        [sin, cos, 0],  # standard rotation matrix
        [0, 0, 1]
    ])

def xyz_radians(radiansX, radiansY):
    cosX, sinX = np.cos(radiansX), np.sin(radiansX)
    cosY, sinY = np.cos(radiansY), np.sin(radiansY)

    rotationX = np.array([
        [1,  0,   0],
        [0, cosX, -sinX],
        [0, sinX,  cosX]
    ])

    rotationY = np.array([
        [ cosY, 0, sinY],
        [  0, 1,  0],
        [-sinY, 0, cosY]
    ])

    return rotationX @ rotationY

def transform(r_matrix, t_vector):  # transform
    transform = np.eye(4)
    transform[:3, :3] = r_matrix
    transform[:3, 3] = t_vector  # Fixed: Use position 3 for translation vector
    return transform

def forward_kinematics(base_angle, shoulder_angle, elbow_angle, link_lengths):
    """
    Calculate forward kinematics for a 3-DOF robot arm
    
    Parameters:
    - base_angle: rotation of the base joint (radians)
    - shoulder_angle: rotation of the shoulder joint (radians)
    - elbow_angle: rotation of the elbow joint (radians)
    - link_lengths: list of link lengths [l1, l2, l3]
    
    Returns:
    - end_effector_position: position of the end effector in world coordinates
    """
    # Base rotation
    R_base = rotation_base(base_angle)
    T_base = transform(R_base, np.array([0, 0, link_lengths[0]]))
    
    # Shoulder rotation (around X-axis) and Elbow rotation (around Y-axis)
    R_arm = xyz_radians(shoulder_angle, 0)
    T_shoulder = transform(R_arm, np.array([0, 0, link_lengths[1]]))
    
    # Elbow rotation
    R_elbow = xyz_radians(0, elbow_angle)
    T_elbow = transform(R_elbow, np.array([0, 0, link_lengths[2]]))
    
    # End effector position
    end_effector_local = np.array([0, 0, 0, 1])  # x,y,z,1 (homogeneous coordinates)
    
    # Calculate global position
    end_effector_global = T_base @ T_shoulder @ T_elbow @ end_effector_local
    
    return end_effector_global[:3]  # Return just the x,y,z components

def print_joint_angles(base_angle, shoulder_angle, elbow_angle):
    """Print the joint angles in both radians and degrees"""
    print(f"Joint Angles:")
    print(f"  Base:     {base_angle:.4f} rad ({np.degrees(base_angle):.2f}°)")
    print(f"  Shoulder: {shoulder_angle:.4f} rad ({np.degrees(shoulder_angle):.2f}°)")
    print(f"  Elbow:    {elbow_angle:.4f} rad ({np.degrees(elbow_angle):.2f}°)")

# Example usage
if __name__ == "__main__":
    # Define link lengths (in whatever units you're using)
    link_lengths = [10, 20, 15]  # Example link lengths
    
    # Define joint angles
    base_angle = np.radians(45)      # Base rotation 45 degrees
    shoulder_angle = np.radians(30)  # Shoulder rotation 30 degrees
    elbow_angle = np.radians(-45)    # Elbow rotation -45 degrees
    
    # Print joint angles
    print_joint_angles(base_angle, shoulder_angle, elbow_angle)
    
    # Calculate end effector position
    end_pos = forward_kinematics(base_angle, shoulder_angle, elbow_angle, link_lengths)
    
    print(f"\nEnd Effector Position: [{end_pos[0]:.2f}, {end_pos[1]:.2f}, {end_pos[2]:.2f}]")
    
    # You can also analyze the workspace by testing different joint angles
    print("\nWorkspace Analysis (Sample Points):")
    angles_to_test = [0, np.pi/4, np.pi/2]
    
    for b in angles_to_test:
        for s in angles_to_test:
            for e in angles_to_test:
                pos = forward_kinematics(b, s, e, link_lengths)
                print(f"  Angles: ({np.degrees(b):.1f}°, {np.degrees(s):.1f}°, {np.degrees(e):.1f}°) → Position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")