package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class CenterTag extends Command {
    private final double kP = 0.4; //.25?
    private final double kI = 0;
    private final double kD = 0;
    private double rotationOutput;
    private double translationOutput;
    private final double maxOutput = 1;
    private Swerve s_Swerve;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/IMX219");
    DoubleSubscriber currentYaw;
    double yawSetpoint = 0;
    DoubleSubscriber currentArea;
    double areaSetPoint = 3;
    BooleanSubscriber hasTarget;
    private PIDController rotationController;
    private PIDController translationController;

    public CenterTag(Swerve subsystem) {
        this.s_Swerve = subsystem;
        currentYaw = table.getDoubleTopic("targetYaw").subscribe(yawSetpoint);
        currentArea = table.getDoubleTopic("targetArea").subscribe(areaSetPoint);
        hasTarget = table.getBooleanTopic("hasTarget").subscribe(false);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Swerve);

        rotationController = new PIDController(kP, kI, kD);
        translationController = new PIDController(kP, kI, kD);
        rotationController.setTolerance(2);
        translationController.setTolerance(4);
      }

    @Override
    public void initialize() {
        rotationController.setSetpoint(yawSetpoint);
        translationController.setSetpoint(areaSetPoint);
    }

    @Override
    public void execute() {
        rotationOutput = MathUtil.clamp(rotationController.calculate(hasTarget.get() ? currentYaw.get() : yawSetpoint), -maxOutput, maxOutput);
        translationOutput = MathUtil.clamp(translationController.calculate(hasTarget.get() ? currentArea.get() : areaSetPoint), -maxOutput, maxOutput);
        s_Swerve.drive(new Translation2d(translationOutput, 0), rotationOutput, false, true);
    }
}
