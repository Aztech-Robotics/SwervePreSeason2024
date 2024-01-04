package frc.lib.swerve;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class DriveMotionPlanner {    
    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
        new PIDConstants(Constants.Drive.kp_translational), 
        new PIDConstants(Constants.Drive.kp_theta, Constants.Drive.ki_theta, Constants.Drive.kd_theta),  
        Constants.Drive.maxVelocity, Math.hypot(Constants.Drive.wheel_base, Constants.Drive.track_width)/2
    ); 
    private PathPlannerTrajectory currentTrajectory; 
    private Double startTime;
    private boolean isTrajectoryFinished = false;

    public DriveMotionPlanner () {}

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
