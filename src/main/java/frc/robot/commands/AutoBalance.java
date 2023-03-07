package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;


public class AutoBalance extends CommandBase{

    private final Swerve s_Swerve;
    private final PIDController forwardController;
    private final double kP = frc.robot.Constants.Swerve.driveKP; //try 0.1
    private final double kI = frc.robot.Constants.Swerve.driveKI; //try 0.1
    private final double kD = frc.robot.Constants.Swerve.driveKP; //try 0.1
    private final double maxOutput = 0.2;
    private double currentAngle;
    private double output;

    public AutoBalance(Swerve subsystem) {
        this.s_Swerve = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Swerve);

        forwardController = new PIDController(kP, kI, kD);
        forwardController.setTolerance(2.0);
      }
    
    @Override
    public void initialize() {
        forwardController.setSetpoint(0);
    }

    @Override
    public void execute() {
        currentAngle = s_Swerve.getPitch();

        output = MathUtil.clamp(forwardController.calculate(currentAngle, 0), -maxOutput, maxOutput);

        s_Swerve.drive(new Translation2d(output, 0), 0, true, true);
    }
}
