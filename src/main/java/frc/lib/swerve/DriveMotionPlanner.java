package frc.lib.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class DriveMotionPlanner {
    private final PIDController x_controller; 
    private final PIDController y_controller;
    private final ProfiledPIDController theta_controller; 
    private final PIDController snapController; 
    private final HolonomicDriveController drive_controller;

    private Trajectory current_trajectory;
    private Rotation2d target_rotation;
    private Double start_time;
    private boolean isTrajectoryFinished = false;

    public DriveMotionPlanner () {
        x_controller = new PIDController(Constants.Drive.kp_x, Constants.Drive.ki_x, Constants.Drive.kd_x);
        y_controller = new PIDController(Constants.Drive.kp_y, Constants.Drive.ki_y, Constants.Drive.kd_y);
        theta_controller = new ProfiledPIDController(Constants.Drive.kp_theta, Constants.Drive.ki_theta, Constants.Drive.kd_theta, 
        new TrapezoidProfile.Constraints(0, 0)); 
        drive_controller = new HolonomicDriveController(x_controller, y_controller, theta_controller);
        snapController = new PIDController(Constants.Drive.kp_snap, Constants.Drive.ki_snap, Constants.Drive.kd_snap); 

        theta_controller.enableContinuousInput(0, 2 * Math.PI);
        snapController.enableContinuousInput(0, 2 * Math.PI);
    }

    public double calculateRotationalAdjustment(double target_heading, double current_heading) {
        if (snapController.getSetpoint() != target_heading) {
            snapController.reset();
            snapController.setSetpoint(target_heading);
        }
        return snapController.calculate(current_heading, target_heading);
    }

    public void setTrajectory (Trajectory trajectory, Rotation2d heading, Pose2d current_pose){
        current_trajectory = trajectory;
        isTrajectoryFinished = false; 
        setTargetRotation(heading);
        start_time = Double.NaN;
        x_controller.reset();
        y_controller.reset();
        theta_controller.reset(current_pose.getRotation().getRadians());
    }

    public void setTargetRotation (Rotation2d target_rotation){
        this.target_rotation = target_rotation;
    }

    public ChassisSpeeds update (Pose2d current_pose, double current_time){
        ChassisSpeeds desired_ChassisSpeeds = null;
        if (current_trajectory != null){
            if (start_time.isNaN()){
                start_time = Timer.getFPGATimestamp();
            }
            double seconds = current_time - start_time;
            Trajectory.State desired_state;
            if (seconds < current_trajectory.getTotalTimeSeconds()){
                desired_state = current_trajectory.sample(seconds);
                desired_ChassisSpeeds = drive_controller.calculate(current_pose, desired_state, target_rotation);
            } else {
                isTrajectoryFinished = true;
                current_trajectory = null;
                desired_ChassisSpeeds = new ChassisSpeeds();
            }
        } 
        return desired_ChassisSpeeds;
    }

    public Translation2d getTranslationalError (Pose2d current_pose, double current_time){
        if (current_trajectory != null && !start_time.isNaN()){
            return current_pose.relativeTo(current_trajectory.sample(current_time - start_time).poseMeters).getTranslation();
        }
        else {
            return new Translation2d();
        }
    }

    public Rotation2d getRotationalError (Rotation2d current_rotation){
        if (target_rotation != null){
            return target_rotation.minus(current_rotation);
        } else {
            return new Rotation2d();
        }
    }

    public boolean isTrajectoryFinished() {
        return isTrajectoryFinished;
    }
}
