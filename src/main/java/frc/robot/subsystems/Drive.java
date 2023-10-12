package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants.SwerveModules;

public class Drive extends SubsystemBase {
  private static Drive mDrive;
  private SwerveModule[] swerveModules;
  private Drive() {
    swerveModules = new SwerveModule[] {
      new SwerveModule(SwerveModules.MOD0, 0),
      new SwerveModule(SwerveModules.MOD1, 1),
      new SwerveModule(SwerveModules.MOD2, 2),
      new SwerveModule(SwerveModules.MOD3, 3)
    };
    for (SwerveModule module : swerveModules){
      module.outputTelemetry();
    }
  }

  public static Drive getInstance () {
    if (mDrive == null) {
      mDrive = new Drive();
    }
    return mDrive;
  }

  @Override
  public void periodic() {
    for (SwerveModule module : swerveModules){
      module.readPeriodicInputs();
      module.writePeriodicOutputs(); 
    }
  }
}
