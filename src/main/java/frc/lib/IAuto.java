package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface IAuto {
    public Command getAutoCommand (); 
    public Pose2d getStartingPose ();
}
