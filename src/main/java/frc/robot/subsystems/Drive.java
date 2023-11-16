package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.ChassisSpeeds;
import frc.lib.swerve.DriveMotionPlanner;
import frc.lib.swerve.ModuleState;
import frc.lib.swerve.SwerveDriveOdometry;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.ControlBoard;
import frc.robot.Telemetry;
import frc.robot.Constants.SwerveModules;
import frc.robot.Constants.Drive.DriveControlMode;
import frc.robot.Constants.Drive.KinematicLimits;

public class Drive extends SubsystemBase {
  private static Drive mDrive;
  private SwerveModule[] swerveModules;
  private Pigeon2 pigeon = new Pigeon2(Constants.Drive.id_pigeon); 

  private PeriodicIO mPeriodicIO = new PeriodicIO(); 
  public enum DriveControlState {
    TeleopControl,
    HeadingControl,
    PathFollowing,
    ForceOrient,
    None
  }
  private DriveControlState mControlState = DriveControlState.None;
  private KinematicLimits mKinematicLimits = Constants.Drive.oneMPSLimits; 
  private DriveMotionPlanner mMotionPlanner = new DriveMotionPlanner();
  private SwerveDriveOdometry mOdometry; 

  private Drive() {
    swerveModules = new SwerveModule[] {
      new SwerveModule(SwerveModules.MOD0, 0),
      new SwerveModule(SwerveModules.MOD1, 1),
      new SwerveModule(SwerveModules.MOD2, 2),
      new SwerveModule(SwerveModules.MOD3, 3)
    };
    pigeon.setYaw(0);
    mOdometry = new SwerveDriveOdometry(Constants.Drive.swerveKinematics, getModulesStates(), new Pose2d()); 
    outputTelemetry();
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
    double yaw_velocity = 0; 
    ModuleState[] meas_module_states = new ModuleState [] {
      new ModuleState(),
      new ModuleState(),
      new ModuleState(),
      new ModuleState()
    };
    ChassisSpeeds meas_chassis_speeds = new ChassisSpeeds(); 
    //Outputs 
    DriveControlMode driveControlMode = DriveControlMode.Velocity; 
    ModuleState[] des_module_states = new ModuleState[] {
      new ModuleState(),
      new ModuleState(),
      new ModuleState(),
      new ModuleState()
    }; 
    ChassisSpeeds des_chassis_speeds = new ChassisSpeeds(); 
    Rotation2d heading_setpoint = new Rotation2d(); 
  }

  public void readPeriodicInputs () {
    mPeriodicIO.timestamp = Timer.getFPGATimestamp(); 
    StatusSignal<Double> yawAngle = pigeon.getYaw(); 
    yawAngle.refresh();
    mPeriodicIO.yawAngle = Rotation2d.fromDegrees(yawAngle.getValue()); 
    StatusSignal<Double> yawVel = pigeon.getAngularVelocityZ(); 
    yawVel.refresh(); 
    mPeriodicIO.yaw_velocity = yawVel.getValue(); 
    for (SwerveModule module : swerveModules) {
      module.readPeriodicInputs();
    }
    mPeriodicIO.meas_module_states = getModulesStates(); 
    mPeriodicIO.meas_chassis_speeds = Constants.Drive.swerveKinematics.toChassisSpeeds(mPeriodicIO.meas_module_states);
    mOdometry.update(mPeriodicIO.yawAngle, mPeriodicIO.meas_module_states);
  }

  public void writePeriodicOutputs () {
    updateSetpoint();
    setModulesStates(mPeriodicIO.des_module_states); 
    for (SwerveModule module : swerveModules) {
      module.writePeriodicOutputs();
    }
  }
  
  @Override
  public void periodic() {
    readPeriodicInputs();
    if (mControlState == DriveControlState.TeleopControl || mControlState == DriveControlState.HeadingControl) {
      mPeriodicIO.des_chassis_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        ControlBoard.getLeftYC0().getAsDouble() * mKinematicLimits.kMaxDriveVelocity, 
        ControlBoard.getLeftXC0().getAsDouble() * mKinematicLimits.kMaxDriveVelocity, 
        ControlBoard.getRightXC0().getAsDouble() * mKinematicLimits.kMaxAngularVelocity
      );
      if (mControlState == DriveControlState.HeadingControl) {
        if (mMotionPlanner.isAtHeadingSetpoint()) {
          mControlState = DriveControlState.TeleopControl; 
        } else {
          mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond = 
          mMotionPlanner.calculateRotationalAdjustment(mPeriodicIO.heading_setpoint.getRadians(), mPeriodicIO.yawAngle.getRadians());
        }
      }
    } else if (mControlState == DriveControlState.PathFollowing) {
      mPeriodicIO.des_chassis_speeds = mMotionPlanner.update(getPose(), mPeriodicIO.timestamp);
    }
    writePeriodicOutputs();
  }

  private void updateSetpoint () {
    Pose2d robot_pose_vel = new Pose2d(
      mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt, 
      mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt, 
      Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt)
    );
    Twist2d twist_vel = new Pose2d().log(robot_pose_vel);
    ChassisSpeeds wanted_speeds = new ChassisSpeeds(
      twist_vel.dx / Constants.kLooperDt, 
      twist_vel.dy / Constants.kLooperDt, 
      twist_vel.dtheta / Constants.kLooperDt
    );
    if (mControlState == DriveControlState.TeleopControl || mControlState == DriveControlState.HeadingControl) {
      // Limit rotational velocity
      wanted_speeds.omegaRadiansPerSecond = Math.signum(wanted_speeds.omegaRadiansPerSecond) * Math.min(mKinematicLimits.kMaxAngularVelocity, Math.abs(wanted_speeds.omegaRadiansPerSecond));
      // Limit translational velocity
      double velocity_magnitude = Math.hypot(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond, mPeriodicIO.des_chassis_speeds.vyMetersPerSecond);
      if (velocity_magnitude > mKinematicLimits.kMaxDriveVelocity) {
        wanted_speeds.vxMetersPerSecond = (wanted_speeds.vxMetersPerSecond / velocity_magnitude) * mKinematicLimits.kMaxDriveVelocity;
        wanted_speeds.vyMetersPerSecond = (wanted_speeds.vyMetersPerSecond / velocity_magnitude) * mKinematicLimits.kMaxDriveVelocity;
      }
      ModuleState[] prev_module_states = mPeriodicIO.des_module_states.clone(); // Get last setpoint to get differentials
      ChassisSpeeds prev_chassis_speeds = Constants.Drive.swerveKinematics.toChassisSpeeds(prev_module_states); 
  
      double dx = wanted_speeds.vxMetersPerSecond - prev_chassis_speeds.vxMetersPerSecond;
      double dy = wanted_speeds.vyMetersPerSecond - prev_chassis_speeds.vyMetersPerSecond;
      double domega = wanted_speeds.omegaRadiansPerSecond - prev_chassis_speeds.omegaRadiansPerSecond;
  
      double max_velocity_step = mKinematicLimits.kMaxAccel * Constants.kLooperDt;
      double min_translational_scalar = 1.0;
      if (max_velocity_step < Double.MAX_VALUE * Constants.kLooperDt) {
        // Check X
        double x_norm = Math.abs(dx / max_velocity_step);
        min_translational_scalar = Math.min(min_translational_scalar, x_norm);
        // Check Y
        double y_norm = Math.abs(dy / max_velocity_step);
        min_translational_scalar = Math.min(min_translational_scalar, y_norm);
  
        min_translational_scalar *= max_velocity_step;
      }
  
      double max_omega_step = mKinematicLimits.kMaxAngularAccel * Constants.kLooperDt;
      double min_omega_scalar = 1.0;
      if (max_omega_step < Double.MAX_VALUE * Constants.kLooperDt) {
        double omega_norm = Math.abs(domega / max_omega_step);
        min_omega_scalar = Math.min(min_omega_scalar, omega_norm);
  
        min_omega_scalar *= max_omega_step;
      }
  
      wanted_speeds = new ChassisSpeeds(
        prev_chassis_speeds.vxMetersPerSecond + dx * min_translational_scalar, 
        prev_chassis_speeds.vyMetersPerSecond + dy * min_translational_scalar, 
        prev_chassis_speeds.omegaRadiansPerSecond + domega * min_omega_scalar
      );
  
      ModuleState[] real_module_setpoints = Constants.Drive.swerveKinematics.toModuleStates(wanted_speeds);
      mPeriodicIO.des_module_states = real_module_setpoints;

    } else if (mControlState == DriveControlState.PathFollowing) {
      mPeriodicIO.des_module_states = Constants.Drive.swerveKinematics.toModuleStates(wanted_speeds); 

    } else if (mControlState == DriveControlState.ForceOrient) {
      mPeriodicIO.des_module_states = new ModuleState [] {
        ModuleState.fromSpeeds(Rotation2d.fromDegrees(-45), 0),
        ModuleState.fromSpeeds(Rotation2d.fromDegrees(45), 0),
        ModuleState.fromSpeeds(Rotation2d.fromDegrees(45), 0),
        ModuleState.fromSpeeds(Rotation2d.fromDegrees(-45), 0)
      };
    }
  }

  private void setModulesStates (ModuleState[] modulesStates) {
    if (mPeriodicIO.driveControlMode == DriveControlMode.PercentOutput) {
      for (ModuleState modState : modulesStates) {
        modState.speedMetersPerSecond = modState.speedMetersPerSecond / mKinematicLimits.kMaxDriveVelocity; 
      }
    }
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setModuleState(modulesStates[i], mPeriodicIO.driveControlMode);
    }
  }

  private ModuleState[] getModulesStates () {
    ModuleState[] moduleStates = new ModuleState[4]; 
    for (int i = 0; i < 4; i++){
      moduleStates[i] = swerveModules[i].getModuleState();
    }
    return moduleStates; 
  }

  public void setDesiredChassisSpeeds (ChassisSpeeds chassisSpeeds) {
    mPeriodicIO.des_chassis_speeds = chassisSpeeds; 
  }

  public void resetGyro () {
    pigeon.reset(); 
  }

  public void setYawAngle (double angle) {
    pigeon.setYaw(angle); 
  }

  public Rotation2d getYawAngle () {
    return mPeriodicIO.yawAngle; 
  }

  public void toggleDriveControl (){
    mPeriodicIO.driveControlMode = mPeriodicIO.driveControlMode == DriveControlMode.PercentOutput? 
    DriveControlMode.Velocity : DriveControlMode.PercentOutput; 
  }

  public void setDriveControlState (DriveControlState state) {
    if (mControlState != state) mControlState = state; 
  }

  public void setKinematicsLimits (KinematicLimits limits) {
    mKinematicLimits = limits; 
  }

  public void resetOdometry (Pose2d pose) {
    for (SwerveModule module : swerveModules) {
      module.resetModule();
    }
    mOdometry.resetPosition(getModulesStates(), pose);
    setYawAngle(pose.getRotation().getDegrees()); 
  }

  public Pose2d getPose () {
    return mOdometry.getPoseMeters(); 
  }

  public void setTrajectory (Trajectory trajectory, Rotation2d heading) {
    mMotionPlanner.setTrajectory(trajectory, heading, getPose());
    mControlState = DriveControlState.PathFollowing; 
  }

  public boolean isTrajectoryFinished () {
    return mControlState == DriveControlState.PathFollowing && mMotionPlanner.isTrajectoryFinished(); 
  }

  public void setHeadingControl (Rotation2d headingSetpoint) {
    mPeriodicIO.heading_setpoint = headingSetpoint; 
    mControlState = DriveControlState.HeadingControl; 
  }

  public void outputTelemetry (){
    Telemetry.swerveTab.addDouble("Yaw Angle", () -> mPeriodicIO.yawAngle.getDegrees()).withPosition(8, 0); 
    Telemetry.swerveTab.addDouble("Yaw Angular Velocity", () -> mPeriodicIO.yaw_velocity).withPosition(9, 0);
  }
}