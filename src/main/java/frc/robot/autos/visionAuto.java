package frc.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class visionAuto extends SequentialCommandGroup {
  DoubleSubscriber areasSub;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/IMX219");

  private double forwardAmount;
  
  public visionAuto(Swerve s_Swerve) {
    areasSub = table.getDoubleTopic("targetArea").subscribe(10);

/*     double area = areasSub.get();
    if(area <= 0) {
        forwardAmount = 0;
    } else if(area <= 6.5) {
        forwardAmount = 0.2;
    } else if (area >= 7.5) {
        forwardAmount = -0.2;
    }
    System.out.println(area); */
    
    addCommands(
        new RepeatCommand(new InstantCommand(() -> 
            s_Swerve.drive(new Translation2d(doMath(areasSub.get()), 0).times(Constants.Swerve.maxSpeed), 0, true, true), s_Swerve))
        );
  }

  private double doMath(double area) {
    if(area <= 0) {
        forwardAmount = 0;
    } else if(area <= 5.5) {
        forwardAmount = 0.2/*  * (5.5-area) */;
    } else if (area >= 6.5) {
        forwardAmount = -0.2/*  * (area-6.5) */;
    } else {
        forwardAmount = 0;
    }
    return forwardAmount;
  }
}
