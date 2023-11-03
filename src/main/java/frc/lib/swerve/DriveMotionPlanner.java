package frc.lib.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class DriveMotionPlanner {
    private final PIDController xController; 
    private final PIDController yController;
    private final ProfiledPIDController thetaController; 
    private final PIDController snapController; 
    private final HolonomicDriveController mDriveController;

    private Trajectory currentTrajectory;
    private Rotation2d targetRotation; 
    private Double startTime;
    private boolean isTrajectoryFinished = false; 

    public DriveMotionPlanner () {
        xController = new PIDController(0, 0, 0);
        yController = new PIDController(0, 0, 0);
        thetaController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)); 
        snapController = new PIDController(0, 0, 0); 
        mDriveController = new HolonomicDriveController(xController, yController, thetaController);

        thetaController.enableContinuousInput(0, 2 * Math.PI);
        snapController.enableContinuousInput(0, 2 * Math.PI);
    }
}
