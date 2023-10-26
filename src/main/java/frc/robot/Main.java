package frc.robot;

import java.util.Scanner;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.swerve.ChassisSpeeds;
import frc.lib.swerve.ModuleState;
import frc.robot.subsystems.Drive;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    // RobotBase.startRobot(Robot::new);
    Scanner input = new Scanner(System.in); 
    while (true) {
      double velX = input.nextDouble();
      double velY = input.nextDouble(); 
      double velTheta = input.nextDouble(); 
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(velY, velX, velTheta); 
      ModuleState[] modulesStates = new ModuleState[4]; 
      modulesStates = Constants.Drive.swerveKinematics.toModuleStates(chassisSpeeds); 
      for (int i=0; i<modulesStates.length; i++) {
        System.out.println("MOD" + i + ": Vel " + modulesStates[i].speedMetersPerSecond + "Angle " + modulesStates[i].angle.getDegrees());
      }
    }
  }
}
