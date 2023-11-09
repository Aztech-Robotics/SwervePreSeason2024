package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Drive mDrive;

  @Override
  public void robotInit() {
    mDrive = Drive.getInstance(); 
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
    m_autonomousCommand = null;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    mDrive.setDriveControlState(DriveControlState.TeleopControl); 
  }

  @Override
  public void teleopPeriodic() {
    if (ControlBoard.driver.getAButtonPressed()) {
      mDrive.setYawAngle(0); 
    }
    if (ControlBoard.driver.getYButtonPressed()) {
      mDrive.setDriveControlState(DriveControlState.ForceOrient);
    } else if (ControlBoard.driver.getYButtonReleased()) {
      mDrive.setDriveControlState(DriveControlState.TeleopControl);
    }
    if (ControlBoard.driver.getBButtonPressed()) {
      mDrive.setKinematicsLimits(Constants.Drive.twoMPSLimits);
    } else if (ControlBoard.driver.getXButtonPressed()) {
      mDrive.setKinematicsLimits(Constants.Drive.oneMPSLimits);
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
