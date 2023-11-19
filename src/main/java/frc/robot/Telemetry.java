package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.IAuto;
import frc.robot.autos.AutoTest;

public class Telemetry {
    public static ShuffleboardTab swerveTab = Shuffleboard.getTab("SwerveData"); 
    public static SendableChooser<IAuto> autoChooser = new SendableChooser<>(); 

    public static void displayAutos () {
        AutoTest autoTest = new AutoTest(); 
        autoChooser.setDefaultOption("None", null);
        autoChooser.addOption("AutoTest", autoTest);
        swerveTab.add(autoChooser); 
    }
}
