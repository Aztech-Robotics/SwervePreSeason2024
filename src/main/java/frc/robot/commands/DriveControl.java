package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.swerve.ChassisSpeeds;
import frc.robot.ControlBoard;
import frc.robot.subsystems.Drive;

public class DriveControl extends CommandBase {
  private final Drive mDrive = Drive.getInstance(); 
  private Supplier<Double> xControl = ControlBoard.getLeftXC0(); 
  private Supplier<Double> yControl = ControlBoard.getLeftYC0();
  private Supplier<Double> thetaControl = ControlBoard.getRightXC0(); 
  public DriveControl() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    mDrive.setDesiredChassisSpeeds(
      new ChassisSpeeds(
        yControl.get(), 
        xControl.get(), 
        thetaControl.get()
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
