package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class FollowPath extends CommandBase {
  private final Drive mDrive = Drive.getInstance(); 
  private PathPlannerTrajectory trajectory;

  public FollowPath (PathPlannerTrajectory trajectory) {
    this.trajectory = trajectory; 
  }

  @Override
  public void initialize() {
    if (mDrive.isReadyForAuto()) {
      mDrive.setTrajectory(trajectory); 
    }
  }

  @Override
  public boolean isFinished() {
    return mDrive.isTrajectoryFinished();
  }
}
