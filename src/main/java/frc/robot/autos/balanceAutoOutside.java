package frc.robot.autos;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Swerve;

public class balanceAutoOutside extends SequentialCommandGroup{
    public balanceAutoOutside(Swerve s_Swerve) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("balanceAutoOutside", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("balance", new AutoBalance(s_Swerve));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose, // Pose2d supplier
            s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
        );
        addCommands(
            autoBuilder.fullAuto(pathGroup)
        );
        
    }
}
