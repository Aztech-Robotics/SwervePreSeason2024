package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.IAuto;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlState;

public class Robot extends TimedRobot {
  private Drive mDrive; 
  private Command mAutonomousCommand;

  @Override
  public void robotInit() {
    mDrive = Drive.getInstance(); 
    Telemetry.displayAutos();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    mDrive.setDriveControlState(DriveControlState.None);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    IAuto autoSelected = Telemetry.autoChooser.getSelected(); 
    if (autoSelected != null) {
      mDrive.setKinematicsLimits(Constants.Drive.uncappedLimits);
      mDrive.resetOdometry(autoSelected.getStartingPose()); 
      mAutonomousCommand = autoSelected.getAutoCommand(); 
      mAutonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (mAutonomousCommand != null) {
      mAutonomousCommand.cancel();
    }
    mDrive.setKinematicsLimits(Constants.Drive.oneMPSLimits); 
    mDrive.setDriveControlState(DriveControlState.TeleopControl); 
  }

  @Override
  public void teleopPeriodic() {
    if (ControlBoard.driver.getAButtonPressed()) {
      mDrive.setYawAngle(0); 
    }
    if (ControlBoard.driver.getBButtonPressed()) {
      mDrive.setDriveControlState(DriveControlState.ForceOrient);
    } else if (ControlBoard.driver.getBButtonReleased()) {
      mDrive.setDriveControlState(DriveControlState.TeleopControl);
    }
    if (ControlBoard.driver.getPOV() != -1) {
      mDrive.setHeadingControl(Rotation2d.fromDegrees(ControlBoard.driver.getPOV()));
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {}
}
