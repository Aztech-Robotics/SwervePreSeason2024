package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.swerve.SwerveDriveKinematics;
import frc.lib.swerve.SwerveModule.SwerveModuleConstants;

public final class Constants {
    public static double kLooperDt = 0.02; 
    public static class Drive {
        public static final int id_pigeon = 13;
        public static final double track_width = Units.inchesToMeters(25); 
        public static final double wheel_base = Units.inchesToMeters(25); 
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

        public static final double maxVelocity = 4.4; 
        public static final double maxAngularVelocity = 9;

        public static class KinematicLimits {
            public double kMaxDriveVelocity = maxVelocity; // m/s
            public double kMaxAccel = Double.MAX_VALUE; // m/s^2
            public double kMaxAngularVelocity = maxAngularVelocity; // rad/s
            public double kMaxAngularAccel = Double.MAX_VALUE; // rad/s^2
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
        public static final KinematicLimits defaultLimits = new KinematicLimits();
    }
    public static class SwerveModules {
        public static final double steering_gear_ratio = 12.8; 
        public static final double drive_gear_ratio = 6.75;
        public static final double wheelCircumference = Math.PI * Units.inchesToMeters(4);

        public static final SwerveModuleConstants MOD0 = new SwerveModuleConstants(1, 2, 3, 331);
        public static final SwerveModuleConstants MOD1 = new SwerveModuleConstants(4, 5, 6, 312);
        public static final SwerveModuleConstants MOD2 = new SwerveModuleConstants(7, 8, 9, 275.8); 
        public static final SwerveModuleConstants MOD3 = new SwerveModuleConstants(10, 11, 12, 0.5); 

        public static final double steer_kP = 1.6;
        public static final double steer_kI = 0;
        public static final double steer_kD = 0;
        public static final double steer_kS = 0;
        public static final double steer_kV = 0;
        
        public static final double drive_kP = 0;
        public static final double drive_kI = 0;
        public static final double drive_kD = 0;
        public static final double drive_kFF = 0;

        public static final double posCoefficient = wheelCircumference / drive_gear_ratio;
        public static final double velCoefficient = posCoefficient / 60; 
    }
}
