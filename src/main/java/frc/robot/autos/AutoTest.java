package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.IAuto;
import frc.robot.commands.FollowPath;

public class AutoTest implements IAuto {
    private final PathPlannerPath samplePath1;
    private final FollowPath commSamplePath1;

    public AutoTest () {
        samplePath1 = PathPlannerPath.fromPathFile("SamplePath1"); 
        commSamplePath1 = new FollowPath(new PathPlannerTrajectory(samplePath1, new ChassisSpeeds())); 
    }

    @Override
    public Command getAutoCommand () {
        return new SequentialCommandGroup( 
            commSamplePath1
        ); 
    }
    
    @Override 
    public Pose2d getStartingPose () {
        return samplePath1.getPreviewStartingHolonomicPose(); 
    }
}
