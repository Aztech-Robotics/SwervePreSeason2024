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
    private final PathPlannerPath samplePath2;
    private final PathPlannerPath samplePath3;
    private final PathPlannerPath samplePath4;

    private final FollowPath commSamplePath1;
    private final FollowPath commSamplePath2;
    private final FollowPath commSamplePath3;
    private final FollowPath commSamplePath4;

    public AutoTest () {
        samplePath1 = PathPlannerPath.fromPathFile("SamplePath5"); 
        samplePath2 = PathPlannerPath.fromPathFile("SamplePath2"); 
        samplePath3 = PathPlannerPath.fromPathFile("SamplePath3"); 
        samplePath4 = PathPlannerPath.fromPathFile("SamplePath4"); 
        commSamplePath1 = new FollowPath(new PathPlannerTrajectory(samplePath1, new ChassisSpeeds())); 
        commSamplePath2 = new FollowPath(new PathPlannerTrajectory(samplePath2, new ChassisSpeeds())); 
        commSamplePath3 = new FollowPath(new PathPlannerTrajectory(samplePath3, new ChassisSpeeds())); 
        commSamplePath4 = new FollowPath(new PathPlannerTrajectory(samplePath4, new ChassisSpeeds())); 
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
