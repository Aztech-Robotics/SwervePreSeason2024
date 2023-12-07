package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.swerve.SwerveModule.SwerveModuleConstants;

public final class Constants {

    public static double kLooperDt = 0.02; 
    public static class Drive {
        public static final int id_pigeon = 13;
        public static final double track_width = 0.57; 
        public static final double wheel_base = 0.57; 
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheel_base / 2.0, track_width / 2.0),
            new Translation2d(wheel_base / 2.0, -track_width / 2.0),
            new Translation2d(-wheel_base / 2.0, track_width / 2.0),
            new Translation2d(-wheel_base / 2.0, -track_width / 2.0)
        );

        public enum DriveControlMode {
            Velocity,
            PercentOutput
        }

        public static final double maxVelocity = 4; //Empirical Max Velocity
        public static final double maxAngularVelocity = 8.9; //Theoretical Max Angular Velocity

        public static class KinematicLimits {
            public double kMaxDriveVelocity = maxVelocity; 
            public double kMaxAccel = Double.MAX_VALUE; 
            public double kMaxAngularVelocity = maxAngularVelocity; 
            public double kMaxAngularAccel = Double.MAX_VALUE; 
            KinematicLimits () {}
            KinematicLimits (double kMaxDriveVelocity, double kMaxAngularVelocity) { 
              this.kMaxDriveVelocity = kMaxDriveVelocity; 
              this.kMaxAngularVelocity = kMaxAngularVelocity; 
            }
            KinematicLimits (double kMaxDriveVelocity, double kMaxAccel, double kMaxAngularVelocity, double kMaxAngularAccel){
              this.kMaxDriveVelocity = kMaxDriveVelocity; 
              this.kMaxAccel = kMaxAccel;
              this.kMaxAngularVelocity = kMaxAngularVelocity; 
              this.kMaxAngularAccel = kMaxAngularAccel;
            }
        } 
        
        public static final KinematicLimits uncappedLimits = new KinematicLimits();
        public static final KinematicLimits autoLimits = new KinematicLimits(3.5, 4, Math.PI, Math.PI);  
        public static final KinematicLimits oneMPSLimits = new KinematicLimits(3, Math.PI);  

        public static final double kp_translational = 0;
        public static final double ki_translational = 0;
        public static final double kd_translational = 0;
        
        public static final double kp_theta = 4.75;
        public static final double ki_theta = 0;
        public static final double kd_theta = 0.11; 

        public static final double kp_snap = 0;
        public static final double ki_snap = 0;
        public static final double kd_snap = 0; 
    }

    public static class SwerveModules {
        public static final double steering_gear_ratio = 12.8; 
        public static final double drive_gear_ratio = 6.75;
        public static final double wheelCircumference = Math.PI * Units.inchesToMeters(4);

        public static final SwerveModuleConstants MOD0 = new SwerveModuleConstants(1, 2, 3, 33);
        public static final SwerveModuleConstants MOD1 = new SwerveModuleConstants(4, 5, 6, 42.5);
        public static final SwerveModuleConstants MOD2 = new SwerveModuleConstants(7, 8, 9, 83.6); 
        public static final SwerveModuleConstants MOD3 = new SwerveModuleConstants(10, 11, 12, 0); 

        public static final double steer_kP = 1.28;
        public static final double steer_kI = 0.5; //0.155 .5
        public static final double steer_kD = 0;
        public static final double steer_kS = 0.5;
        public static final double steer_kV = 0;
        
        public static final double drive_kP = 1.82e-4;
        public static final double drive_kI = 0;
        public static final double drive_kD = 0;
        public static final double drive_kFF = 0;

        public static final double posCoefficient = wheelCircumference / drive_gear_ratio;
        public static final double velCoefficient = posCoefficient / 60; 
    }
}
