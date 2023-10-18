package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.swerve.ChassisSpeeds;
import frc.robot.ControlBoard;
import frc.robot.subsystems.Drive;

public class DriveControl extends CommandBase {
  private final Drive mDrive = Drive.getInstance(); 
  private DoubleSupplier xControl = ControlBoard.getLeftXC0(); 
  private DoubleSupplier yControl = ControlBoard.getLeftYC0();
  private DoubleSupplier thetaControl = ControlBoard.getRightXC0(); 
  public DriveControl() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    mDrive.setDesiredChassisSpeeds(
      new ChassisSpeeds(
        yControl.getAsDouble(), 
        xControl.getAsDouble(), 
        thetaControl.getAsDouble() * 0.25
      )
    );
  }

  @Override
  public void end(boolean interrupted) {
    mDrive.setDesiredChassisSpeeds(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
