package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.AutoBalance;
import java.util.List;

public class balanceAuto extends SequentialCommandGroup {
  public balanceAuto(Swerve s_Swerve) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(0.5, 0),
                new Translation2d(1, 0),
                new Translation2d(1.5, 0),
                new Translation2d(2, 0),
                new Translation2d(3, 0),
                new Translation2d(4, 0),
                new Translation2d(4.5, 0)
                ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4.5, 1.8, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);   

    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())), //prep for trajectory
        swerveControllerCommand, //follow trajectory onto charging station
        new AutoBalance(s_Swerve), //balance on charging station, assuming robot is partially on there already
        new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0.01), 0, true, true))); //drive sideways slightly as to lock wheels parallel with charging station
  }
}