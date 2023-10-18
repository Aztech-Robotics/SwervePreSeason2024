package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.ChassisSpeeds;
import frc.lib.swerve.ModuleState;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModules;
import frc.robot.Constants.Drive.DriveControlMode;

public class Drive extends SubsystemBase {
  private static Drive mDrive;
  private SwerveModule[] swerveModules;
  private Pigeon2 pigeon = new Pigeon2(Constants.Drive.id_pigeon); 

  private PeriodicIO mPeriodicIO = new PeriodicIO(); 

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

  public static class PeriodicIO {
    //Inputs
    double timestamp = 0; 
    Rotation2d yawAngle = new Rotation2d(); 
    ModuleState[] meas_module_states = new ModuleState [] {
      new ModuleState(),
      new ModuleState(),
      new ModuleState(),
      new ModuleState()
    };
    ChassisSpeeds meas_chassis_speeds = new ChassisSpeeds(); 
    //Outputs 
    DriveControlMode driveControlMode = DriveControlMode.PercentOutput; 
    ModuleState[] des_module_states = new ModuleState[] {
      new ModuleState(),
      new ModuleState(),
      new ModuleState(),
      new ModuleState()
    }; 
    ChassisSpeeds des_chassis_speeds = new ChassisSpeeds(); 
  }

  public void readPeriodicInputs () {
    mPeriodicIO.timestamp = Timer.getFPGATimestamp(); 
    StatusSignal<Double> yawAngle = pigeon.getYaw(); 
    yawAngle.refresh();
    mPeriodicIO.yawAngle = Rotation2d.fromDegrees(yawAngle.getValue()); 
    for (SwerveModule module : swerveModules) {
      module.readPeriodicInputs();
    }
    mPeriodicIO.meas_module_states = getModulesStates(); 
    mPeriodicIO.meas_chassis_speeds = Constants.Drive.swerveKinematics.toChassisSpeeds(mPeriodicIO.meas_module_states);
  }

  public void writePeriodicOutputs () {
    mPeriodicIO.des_module_states = Constants.Drive.swerveKinematics.toModuleStates(mPeriodicIO.des_chassis_speeds); 
    setModulesStates(mPeriodicIO.des_module_states); 
    for (SwerveModule module : swerveModules) {
      module.writePeriodicOutputs();
    }
  }

  @Override
  public void periodic() {
    readPeriodicInputs();
    writePeriodicOutputs();
  }

  public void setModulesStates (ModuleState[] modulesStates) {
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setModuleState(modulesStates[i], mPeriodicIO.driveControlMode);
    }
  }

  public ModuleState[] getModulesStates () {
    ModuleState[] moduleStates = new ModuleState[4]; 
    for (int i = 0; i < 4; i++){
      moduleStates[i] = swerveModules[i].getModuleState();
    }
    return moduleStates; 
  }

  public void setDesiredChassisSpeeds (ChassisSpeeds chassisSpeeds) {
    mPeriodicIO.des_chassis_speeds = chassisSpeeds; 
  }

  public CommandBase toggleDriveControl (){
    return runOnce(
      () -> {
        mPeriodicIO.driveControlMode = mPeriodicIO.driveControlMode == DriveControlMode.PercentOutput? 
        DriveControlMode.Velocity : DriveControlMode.PercentOutput; 
      }
    ); 
  }
}
