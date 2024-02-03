package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.lightPatterns;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Swerve;

public class CenterNote extends Command {
    private final double kP = 0.4; //.25?
    private final double kI = 0;
    private final double kD = 0;
    private double strafeOutput;
    private double translationOutput;
    private final double maxOutput = 1;
    private Swerve s_Swerve;
    private Lights s_Lights;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Microsoft_LifeCam_HD-3000");
    DoubleSubscriber currentYaw;
    double yawSetpoint = 0;
    DoubleSubscriber currentArea;
    double areaSetPoint = 3;
    BooleanSubscriber hasTarget;
    private PIDController strafeController;
    private PIDController translationController;

    public CenterNote(Swerve subsystem, Lights s_Lights) {
        this.s_Swerve = subsystem;
        this.s_Lights = s_Lights;
        currentYaw = table.getDoubleTopic("targetYaw").subscribe(yawSetpoint);
        currentArea = table.getDoubleTopic("targetArea").subscribe(areaSetPoint);
        hasTarget = table.getBooleanTopic("hasTarget").subscribe(false);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Swerve, s_Lights);

        strafeController = new PIDController(kP, kI, kD);
        translationController = new PIDController(kP, kI, kD);
        strafeController.setTolerance(2);
        translationController.setTolerance(4);
      }

    @Override
    public void initialize() {
        strafeController.setSetpoint(yawSetpoint);
        translationController.setSetpoint(areaSetPoint);
        SmartDashboard.putBoolean("AutoHasTarget", hasTarget.get());
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("AutoHasTarget", hasTarget.get());
        if(hasTarget.get()) {
            s_Lights.custom(lightPatterns.strobeGold);
        } else {
            s_Lights.custom(lightPatterns.blue);
        }
        strafeOutput = MathUtil.clamp(strafeController.calculate(hasTarget.get() ? currentYaw.get() : yawSetpoint), -maxOutput, maxOutput);
        translationOutput = MathUtil.clamp(translationController.calculate(hasTarget.get() ? currentArea.get() : areaSetPoint), -maxOutput, maxOutput);
        s_Swerve.drive(new Translation2d(translationOutput, strafeOutput), 0, false, true);
    }
}
