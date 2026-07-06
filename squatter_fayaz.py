import numpy as np
from typing import Tuple, List
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt


class Squatter:
    def __init__(self):
        self.total_time = 10  
        self.sampling_time = 0.1  
        self.num_samples = int(self.total_time / self.sampling_time) + 1
        

    @staticmethod
    def calculate_angles(l1,l2,hip_height) -> Tuple[List[np.ndarray], List[np.ndarray], int]:
        
        cos_knee = (l1**2 + l2**2 - hip_height**2) / (2 * l1 * l2)
        knee_angle = np.arccos(cos_knee)  

        
        cos_hip = (l1**2 + hip_height**2 - l2**2) / (2 * l1 * hip_height)
        hip_angle = np.arccos(cos_hip)

        
        hip_angle_deg = np.pi/2 - hip_angle
        knee_angle_deg = knee_angle
        ankle_angle_deg = - (hip_angle_deg + knee_angle_deg)  

        return hip_angle_deg, knee_angle_deg, ankle_angle_deg
    

    @staticmethod
    def generate_proportional(sampling_time, squat_depth_percentage=50.0, 
                                        squat_cycles=4, total_time=10.0):
        
        
        
        reference_hip_angle = 45.0      
        reference_knee_angle = 90.0     
        reference_ankle_angle = 45.0    
        
        
        target_hip_angle = reference_hip_angle * (squat_depth_percentage / 100.0)
        target_knee_angle = reference_knee_angle * (squat_depth_percentage / 100.0)  
        target_ankle_angle = reference_ankle_angle * (squat_depth_percentage / 100.0)
        
        
        deg_to_rad = np.pi / 180.0
        target_hip_rad = target_hip_angle * deg_to_rad
        target_knee_rad = target_knee_angle * deg_to_rad
        target_ankle_rad = target_ankle_angle * deg_to_rad
        
        
        num_samples = int(total_time / sampling_time) + 1
        t = np.linspace(0, total_time, num_samples)
        
        
        frequency = squat_cycles / total_time
        half_sine = 0.5 * (1 - np.cos(2 * np.pi * frequency * t))
        
        
        right_joints = [np.zeros_like(half_sine) for _ in range(6)]
        left_joints = [np.zeros_like(half_sine) for _ in range(6)]
        
        
        
        right_joints[1] = -target_hip_rad * half_sine    
        right_joints[3] = target_knee_rad * half_sine    
        right_joints[4] = -target_ankle_rad * half_sine  
        
        
        left_joints[1] = -target_hip_rad * half_sine     
        left_joints[3] = target_knee_rad * half_sine     
        left_joints[4] = -target_ankle_rad * half_sine   
        
        print(f"Squat depth: {squat_depth_percentage}% of reference squat")
        print(f"Target angles - Hip: {target_hip_angle:.1f}°, Knee: {target_knee_angle:.1f}°, Ankle: {target_ankle_angle:.1f}°")
        print(f"Motion: {squat_cycles} cycles over {total_time}s")
        
        max_range = num_samples
        return right_joints, left_joints, max_range
    
    @staticmethod
    def generate_rigid(sampling_time):
        """Generates joint data with half-sine wave shape starting and ending at zero, in radians."""
        total_time = 10  
        num_samples = int(total_time / sampling_time) + 1
        t = np.linspace(0, total_time, num_samples)

        frequency = 4 / total_time  
        half_sine = 0.5 * (1 - np.cos(2 * np.pi * frequency * t))  

        
        deg_to_rad = np.pi / 180.0

        
        right_joints = [np.zeros_like(half_sine) for _ in range(6)]
        left_joints = [np.zeros_like(half_sine) for _ in range(6)]

        
        right_joints[1] = (-45 * deg_to_rad) * half_sine  
        right_joints[3] = (90 * deg_to_rad) * half_sine  
        right_joints[4] = (-45 * deg_to_rad) * half_sine  

        left_joints[1] = (-45 * deg_to_rad) * half_sine  
        left_joints[3] = (90 * deg_to_rad) * half_sine  
        left_joints[4] = (-45 * deg_to_rad) * half_sine  

        return right_joints, left_joints, num_samples
    

    @staticmethod
    def generate_advanced(sampling_time, squat_depth_percentage=50.0,
                                        squat_cycles=4, total_time=10.0):

        
        l3 = 0.280  
        l4 = 0.260  
        
        
        standing_height = l3 + l4  
        
        
        num_samples = int(total_time / sampling_time) + 1
        t = np.linspace(0, total_time, num_samples)
        frequency = squat_cycles / total_time
        
        
        squat_modulation = 0.5 * (1 - np.cos(2 * np.pi * frequency * t))
        
        
        
        right_joints = [np.zeros_like(squat_modulation) for _ in range(6)]
        left_joints = [np.zeros_like(squat_modulation) for _ in range(6)]
        hip_heights = np.zeros_like(squat_modulation)


        for i, (squat_factor) in enumerate(squat_modulation):
            
            
            
            
            max_squat_reduction = standing_height * (squat_depth_percentage / 100.0) * 0.6  
            current_leg_length = standing_height - max_squat_reduction * squat_factor
            
            print(current_leg_length)
            hip_heights[i] = current_leg_length
        
            
            
            
            
            cos_knee_interior = (l3**2 + l4**2 - current_leg_length**2) / (2 * l3 * l4)
            cos_knee_interior = np.clip(cos_knee_interior, -1, 1)
            
            
            knee_interior_angle = np.arccos(cos_knee_interior)
            
            
            knee_angle = np.pi - knee_interior_angle
            
            
            
            if current_leg_length > 0.001:  
                sin_hip = (l4 * np.sin(knee_interior_angle)) / current_leg_length
                sin_hip = np.clip(sin_hip, -1, 1)
                hip_angle_from_vertical = np.arcsin(sin_hip)
            else:
                hip_angle_from_vertical = 0
            
            
            
            
            hip_angle = hip_angle_from_vertical 
            
            
            
            
            ankle_angle = (knee_angle - hip_angle)  
            

            
            fk_leg_length = l3 * np.cos(-hip_angle) + l4 * np.cos(-hip_angle + knee_angle)
            print(f"Planned: {current_leg_length:.4f}, FK: {fk_leg_length:.4f}, Diff: {fk_leg_length - current_leg_length:.4f}")

        
            
            if i == 0:  
                print(f"Initial angles - Hip: {np.degrees(hip_angle):.2f}°, Knee: {np.degrees(knee_angle):.2f}°, Ankle: {np.degrees(ankle_angle):.2f}°")
            
            
            
            
            
            right_joints[1][i] = -hip_angle    
            right_joints[3][i] = knee_angle    
            right_joints[4][i] = -ankle_angle  
            
            
            left_joints[1][i] = -hip_angle     
            left_joints[3][i] = knee_angle     
            left_joints[4][i] = -ankle_angle   
        
        
        max_angles = {
            'hip': np.degrees(np.max(np.abs([right_joints[1]]))),
            'knee': np.degrees(np.max(right_joints[3])),
            'ankle': np.degrees(np.max(np.abs(right_joints[4])))
        }
        
        print(f"Maximum angles - Hip: ±{max_angles['hip']:.1f}°, Knee: {max_angles['knee']:.1f}°, Ankle: ±{max_angles['ankle']:.1f}°")
        print(f"Final angles - Hip: {np.degrees(right_joints[1][-1]):.2f}°, Knee: {np.degrees(right_joints[3][-1]):.2f}°, Ankle: {np.degrees(right_joints[4][-1]):.2f}°")
        
        max_range = num_samples
        
        
        return right_joints, left_joints, max_range, hip_heights
    

  
    @staticmethod
    def generate_advanced_smoothed(
        sampling_time, squat_depth_percentage=50.0,
        squat_cycles=4, total_time=10.0,
        window_length=21, polyorder=3
    ):
        
        def smooth_with_padding(arr, window_length, polyorder):
            pad = window_length // 2
            arr_padded = np.pad(arr, pad, mode='reflect')
            arr_smooth = savgol_filter(arr_padded, window_length, polyorder, mode='nearest')
            return arr_smooth[pad:-pad]
        

        def quintic_trajectory(t, T, theta_max):
            
            
            
            tau = t / T
            return theta_max * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)

        """
        Generates smooth joint data for squatting using Savitzky-Golay filtering.
        Returns: right_joints, left_joints, num_samples, hip_heights
        """
        l3 = 0.280  
        l4 = 0.260  
        standing_height = l3 + l4

        num_samples = int(total_time / sampling_time) + 1
        t = np.linspace(0, total_time, num_samples)
        frequency = squat_cycles / total_time
        squat_modulation = 0.5 * (1 - np.cos(2 * np.pi * frequency * t))

        right_joints = [np.zeros_like(squat_modulation) for _ in range(6)]
        left_joints = [np.zeros_like(squat_modulation) for _ in range(6)]
        hip_heights = np.zeros_like(squat_modulation)

        for i, squat_factor in enumerate(squat_modulation):
            max_squat_reduction = standing_height * (squat_depth_percentage / 100.0) * 0.6
            current_leg_length = standing_height - max_squat_reduction * squat_factor
            hip_heights[i] = current_leg_length

            cos_knee_interior = (l3**2 + l4**2 - current_leg_length**2) / (2 * l3 * l4)
            cos_knee_interior = np.clip(cos_knee_interior, -1, 1)
            knee_interior_angle = np.arccos(cos_knee_interior)
            knee_angle = np.pi - knee_interior_angle

            if current_leg_length > 0.001:
                sin_hip = (l4 * np.sin(knee_interior_angle)) / current_leg_length
                sin_hip = np.clip(sin_hip, -1, 1)
                hip_angle_from_vertical = np.arcsin(sin_hip)
            else:
                hip_angle_from_vertical = 0

            hip_angle = hip_angle_from_vertical
            ankle_angle = knee_angle - hip_angle

            right_joints[1][i] = -hip_angle
            right_joints[3][i] = knee_angle
            right_joints[4][i] = -ankle_angle

            left_joints[1][i] = -hip_angle
            left_joints[3][i] = knee_angle
            left_joints[4][i] = -ankle_angle

        
        
        if window_length % 2 == 0:
            window_length += 1
        if window_length > num_samples:
            window_length = num_samples if num_samples % 2 == 1 else num_samples - 1
        if polyorder >= window_length:
            polyorder = window_length - 1

        for idx in [1, 3, 4]:
            right_joints[idx] = smooth_with_padding(right_joints[idx], window_length, polyorder)
            left_joints[idx] = smooth_with_padding(left_joints[idx], window_length, polyorder)
            hip_heights = smooth_with_padding(hip_heights, window_length, polyorder)

        
        
        
        
        
        

        
        
        

        
        

        
        
        

        
        
        

        return right_joints, left_joints, num_samples, hip_heights
    
    @staticmethod
    def generate_squat_joint_data(sampling_time):
        """Generates joint data with half-sine wave shape starting and ending at zero, in radians."""
        total_time = 10  
        num_samples = int(total_time / sampling_time) + 1
        t = np.linspace(0, total_time, num_samples)

        frequency = 4 / total_time  
        half_sine = 0.5 * (1 - np.cos(2 * np.pi * frequency * t))  

        
        deg_to_rad = np.pi / 180.0

        
        right_joints = [np.zeros_like(half_sine) for _ in range(6)]
        left_joints = [np.zeros_like(half_sine) for _ in range(6)]

        
        right_joints[1] = (-45 * deg_to_rad) * half_sine  
        right_joints[3] = (90 * deg_to_rad) * half_sine  
        right_joints[4] = (-45 * deg_to_rad) * half_sine  

        left_joints[1] = (-45 * deg_to_rad) * half_sine  
        left_joints[3] = (90 * deg_to_rad) * half_sine  
        left_joints[4] = (-45 * deg_to_rad) * half_sine  

        max_range = num_samples
        return right_joints, left_joints, max_range


if __name__ == "__main__":
    squatter = Squatter()
    
    right_joints, left_joints, num_samples, hip_heights = squatter.generate_advanced(sampling_time=0.1, squat_depth_percentage=50,squat_cycles=4)
    
    
    t = np.linspace(0, 10, num_samples)
    


    _,_,_, new_hip_heights = squatter.generate_advanced(sampling_time=0.1, squat_depth_percentage=100,squat_cycles=4)
    plt.figure(figsize=(10,6))
    plt.subplot(2, 1, 1)
    
    plt.plot(t, np.rad2deg(right_joints[1]), label="Right Hip (joint 2)")
    plt.plot(t, np.rad2deg(right_joints[3]), label="Right Knee (joint 4)")
    plt.plot(t, np.rad2deg(right_joints[4]), label="Right Ankle (joint 5)")
    plt.plot(t, hip_heights*100, label="Hip Heights 50% Squat Depth", linestyle='--')
    plt.plot(t, new_hip_heights*100, label="Hip Heights 100% Squat Depth", linestyle='--')
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (degrees) / Height (cm)")
    plt.title("Right Leg Joint Angles During Squat")
    plt.legend()
    plt.grid()
    plt.savefig("right_leg_squat.png")
    

    r1, l1, num_sam = squatter.generate_squat_joint_data(sampling_time=0.1)
    t = np.linspace(0, 10, num_sam)

    
    plt.subplot(2, 1, 2)
    plt.plot(t, np.rad2deg(-l1[1]), label="Right Hip (joint 2)", linestyle='--')
    plt.plot(t, np.rad2deg(l1[3]), label="Right Knee (joint 4)")
    plt.plot(t, np.rad2deg(l1[4]), label="Right Ankle (joint 5)",  linestyle='--')
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (degrees) / Height (cm)")
    plt.title("Right Leg Joint Angles During Squat")
    plt.legend()
    plt.grid()
    plt.savefig("right_leg_squat.png")



    dt = t[1] - t[0]
    
    hip_angle = right_joints[1]
    knee_angle = right_joints[3]
    ankle_angle = right_joints[4]

    
    hip_vel = np.gradient(hip_angle, dt)
    knee_vel = np.gradient(knee_angle, dt)
    ankle_vel = np.gradient(ankle_angle, dt)

    
    hip_acc = np.gradient(hip_vel, dt)
    knee_acc = np.gradient(knee_vel, dt)
    ankle_acc = np.gradient(ankle_vel, dt)


    
    l3 = 0.280  
    l4 = 0.260  
    y_hip = l3 * np.cos(hip_angle) + l4 * np.cos(hip_angle + knee_angle)  

    
    y_hip_disp = y_hip - y_hip[0]

    
    y_hip_vel = np.gradient(y_hip_disp, dt)
    y_hip_acc = np.gradient(y_hip_vel, dt)

    
    plt.figure(figsize=(10, 8))
    plt.plot(t, hip_heights - hip_heights[0], label="Planned Hip Drop (m)")
    plt.plot(t, y_hip_disp, label="Actual Hip Drop (m)")
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Displacement (m)")
    plt.title("Planned vs Actual Hip Displacement")
    plt.grid()
    


    
    
    
    
    
    
    

    
    
    
    
    
    
    

    plt.tight_layout()
    plt.savefig("hip_displacement_velocity_acceleration.png")
    



    
    plt.figure(figsize=(10, 8))
    plt.subplot(2, 1, 1)
    plt.plot(t, np.rad2deg(hip_vel), label="Hip Velocity (deg/s)")
    plt.plot(t, np.rad2deg(knee_vel), label="Knee Velocity (deg/s)")
    plt.plot(t, np.rad2deg(ankle_vel), label="Ankle Velocity (deg/s)")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (deg/s)")
    plt.title("Joint Angle Velocities During Squat")
    plt.legend()
    plt.grid()

    
    plt.subplot(2, 1, 2)
    plt.plot(t, np.rad2deg(hip_acc), label="Hip Acceleration (deg/s²)")
    plt.plot(t, np.rad2deg(knee_acc), label="Knee Acceleration (deg/s²)")
    plt.plot(t, np.rad2deg(ankle_acc), label="Ankle Acceleration (deg/s²)")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (deg/s²)")
    plt.title("Joint Angle Accelerations During Squat")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.savefig("joint_angle_velocity_acceleration.png")
    plt.show()




