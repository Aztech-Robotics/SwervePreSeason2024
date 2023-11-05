package frc.lib.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.Conversions;
import frc.robot.Constants;
import frc.robot.Telemetry;
import frc.robot.Constants.Drive.DriveControlMode;

public class SwerveModule {
    private final int moduleNumber;
    private final TalonFX mSteerMotor;
    private final CANSparkMax mDriveMotor;
    private final CANcoder mCANcoder; 
    
    private RelativeEncoder driveMotorEncoder; 
    private SparkMaxPIDController drivePIDController;

    private PeriodicIO mPeriodicIO = new PeriodicIO(); 
    private ModuleState targetModuleState; 
    
    public SwerveModule (SwerveModuleConstants moduleConstants, int moduleNumber){
        this.moduleNumber = moduleNumber; 
        mSteerMotor = new TalonFX(moduleConstants.steerMotorID); 
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless); 
        mCANcoder = new CANcoder(moduleConstants.cancoderID); 
        //SteerMotor Config
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs(); 
        currentLimitsConfigs.SupplyCurrentThreshold = 40;
        currentLimitsConfigs.SupplyTimeThreshold = 0.1; 
        currentLimitsConfigs.SupplyCurrentLimit = 25;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs(); 
        feedbackConfigs.FeedbackRemoteSensorID = mCANcoder.getDeviceID();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; 
        feedbackConfigs.RotorToSensorRatio = Constants.SwerveModules.steering_gear_ratio;
        ClosedLoopGeneralConfigs closedLoopConfigs = new ClosedLoopGeneralConfigs(); 
        closedLoopConfigs.ContinuousWrap = true; 
        Slot0Configs slot0Configs = new Slot0Configs(); 
        slot0Configs.kP = Constants.SwerveModules.steer_kP;
        slot0Configs.kI = Constants.SwerveModules.steer_kI;
        slot0Configs.kD = Constants.SwerveModules.steer_kD;
        slot0Configs.kS = Constants.SwerveModules.steer_kS;
        slot0Configs.kV = Constants.SwerveModules.steer_kV; 
        //DriveMotor Config
        driveMotorEncoder = mDriveMotor.getEncoder();
        driveMotorEncoder.setPositionConversionFactor(Constants.SwerveModules.posCoefficient);
        driveMotorEncoder.setVelocityConversionFactor(Constants.SwerveModules.velCoefficient);
        drivePIDController = mDriveMotor.getPIDController(); 
        mDriveMotor.enableVoltageCompensation(12); 
        mDriveMotor.setSmartCurrentLimit(40); 
        mDriveMotor.setInverted(false);
        drivePIDController.setP(Constants.SwerveModules.drive_kP, 0);
        drivePIDController.setI(Constants.SwerveModules.drive_kI, 0);
        drivePIDController.setD(Constants.SwerveModules.drive_kD, 0);
        drivePIDController.setFF(Constants.SwerveModules.drive_kFF, 0); 
        TalonFXConfiguration gral_config = new TalonFXConfiguration();
        gral_config.ClosedLoopGeneral = closedLoopConfigs;
        gral_config.CurrentLimits = currentLimitsConfigs;
        gral_config.Feedback = feedbackConfigs;
        gral_config.Slot0 = slot0Configs; 
        //CANcoder Config
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; 
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive; 
        cancoderConfigs.MagnetSensor.MagnetOffset = Rotation2d.fromDegrees(moduleConstants.angleOffset).getRotations(); 
        //Configs Update 
        mCANcoder.getConfigurator().apply(cancoderConfigs);
        mSteerMotor.getConfigurator().apply(gral_config); 
        mDriveMotor.burnFlash();
        setNeutralMode(true);
    }

    public static class PeriodicIO {
        //Inputs
        double timestamp = 0;
        double currentAngle = 0;
        double velocity = 0; 
        double drivePosition = 0;
        //Outputs
        double rotationDemand = 0;
        double driveDemand = 0; 
        DriveControlMode controlMode = DriveControlMode.PercentOutput; 
    }

    public static class SwerveModuleConstants {
        public final int driveMotorID;
        public final int steerMotorID;
        public final int cancoderID;
        public final double angleOffset;

        public SwerveModuleConstants(int driveMotorID, int steerMotorID, int cancoderID, double angleOffset) {
            this.driveMotorID = driveMotorID;
            this.steerMotorID = steerMotorID;
            this.cancoderID = cancoderID;
            this.angleOffset = angleOffset;
        }
    }

    public void readPeriodicInputs (){
        mPeriodicIO.timestamp = Timer.getFPGATimestamp(); 
        StatusSignal<Double> absPosCancoder = mCANcoder.getAbsolutePosition(); 
        absPosCancoder.refresh(); 
        mPeriodicIO.currentAngle = Rotation2d.fromRotations(absPosCancoder.getValue()).getDegrees();
        mPeriodicIO.velocity = driveMotorEncoder.getVelocity();
        mPeriodicIO.drivePosition = driveMotorEncoder.getPosition(); 
    }

    public void writePeriodicOutputs (){
        if (targetModuleState == null) {
            return;
        }
        double targetAngle = Conversions.signedToUnsignedDeg(targetModuleState.angle.getDegrees()); 
        Rotation2d targetAngleRot = Rotation2d.fromDegrees(targetAngle); 
        double currentAngle = mPeriodicIO.currentAngle;
        double targetVelocity = targetModuleState.speedMetersPerSecond; 
        if (Math.abs(targetAngle - currentAngle) > 90){
            targetAngleRot = targetAngle > 180 ? Rotation2d.fromDegrees(targetAngle - 180) : Rotation2d.fromDegrees(targetAngle + 180); 
            targetVelocity = -targetVelocity; 
        } 
        targetAngleRot = Rotation2d.fromDegrees(Conversions.unsignedToSignedDeg(targetAngleRot.getDegrees())); 
        mPeriodicIO.rotationDemand = targetAngleRot.getRotations();
        mSteerMotor.setControl(new PositionDutyCycle(mPeriodicIO.rotationDemand));
        if (mPeriodicIO.controlMode == DriveControlMode.Velocity){
            mPeriodicIO.driveDemand = targetVelocity / Constants.SwerveModules.velCoefficient; 
            drivePIDController.setReference(mPeriodicIO.driveDemand, ControlType.kVelocity, 0, 0);
        } else {
            mPeriodicIO.driveDemand = targetVelocity;
            drivePIDController.setReference(mPeriodicIO.driveDemand, ControlType.kDutyCycle, 0, 0);
        }
    }

    public void resetModule (){
        driveMotorEncoder.setPosition(0);
    }

    public void setModuleState (ModuleState desiredModuleState, DriveControlMode controlMode) {
        targetModuleState = desiredModuleState; 
        mPeriodicIO.controlMode = controlMode; 
    }

    public ModuleState getModuleState (){
        return new ModuleState(mPeriodicIO.drivePosition, Rotation2d.fromDegrees(mPeriodicIO.currentAngle), mPeriodicIO.velocity); 
    }

    public void setNeutralMode (boolean wantBrake){
        MotorOutputConfigs neutralMode = new MotorOutputConfigs();
        if (wantBrake) {
            neutralMode.NeutralMode = NeutralModeValue.Brake;
            mDriveMotor.setIdleMode(IdleMode.kBrake);
        } else {
            neutralMode.NeutralMode = NeutralModeValue.Coast;
            mDriveMotor.setIdleMode(IdleMode.kCoast);
        }
        mSteerMotor.getConfigurator().apply(neutralMode);
    }

    public void setDriveControlMode (DriveControlMode mode){
        mPeriodicIO.controlMode = DriveControlMode.PercentOutput; 
    }

    public void outputTelemetry (){
        ShuffleboardLayout motorsData = Telemetry.swerveTab.getLayout("Module " + moduleNumber, BuiltInLayouts.kList)
        .withSize(2, 3).withPosition(2 * moduleNumber, 0);
        motorsData.addDouble("Angle", () -> mPeriodicIO.currentAngle);
        motorsData.addDouble("Velocity", ()-> mPeriodicIO.velocity);
        motorsData.addDouble("Position", () -> mPeriodicIO.drivePosition); 
    }
}
