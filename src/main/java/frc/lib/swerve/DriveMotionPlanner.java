package frc.lib.swerve;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class DriveMotionPlanner {    
    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
        new PIDConstants(0), new PIDConstants(0), 
        Constants.Drive.maxVelocity, Math.hypot(Constants.Drive.wheel_base, Constants.Drive.track_width)
    ); 
    private final PIDController snapController = new PIDController(
        Constants.Drive.kp_snap, Constants.Drive.ki_snap, Constants.Drive.kd_snap
    );
    private PathPlannerTrajectory currentTrajectory; 
    private Double startTime;
    private boolean isTrajectoryFinished = false;

    public DriveMotionPlanner () {}

    public double calculateRotationalAdjustment(double target_heading, double current_heading) {
        if (snapController.getSetpoint() != target_heading) {
            snapController.reset();
            snapController.setSetpoint(target_heading);
        }
        return snapController.calculate(current_heading, target_heading);
    }

    public boolean isAtHeadingSetpoint () {
        return snapController.atSetpoint(); 
    }

    public void setTrajectory (PathPlannerTrajectory trajectory, Pose2d current_pose, ChassisSpeeds current_speeds){
        currentTrajectory = trajectory; 
        driveController.reset(current_pose, current_speeds);
        isTrajectoryFinished = false; 
        startTime = Double.NaN;
    }

    public ChassisSpeeds update (Pose2d current_pose, double current_time){
        ChassisSpeeds desired_ChassisSpeeds = new ChassisSpeeds();
        if (currentTrajectory != null){
            if (startTime.isNaN()){
                startTime = Timer.getFPGATimestamp();
            }
            double seconds = current_time - startTime;
            PathPlannerTrajectory.State desired_state; 
            if (seconds < currentTrajectory.getTotalTimeSeconds()){
                desired_state = currentTrajectory.sample(seconds);
            } else {
                desired_state = currentTrajectory.getEndState();
                isTrajectoryFinished = true;
                currentTrajectory = null;
            }
            desired_ChassisSpeeds = driveController.calculateRobotRelativeSpeeds(current_pose, desired_state); 
        } 
        return desired_ChassisSpeeds;
    }

    public boolean isTrajectoryFinished() {
        return isTrajectoryFinished;
    }
}
