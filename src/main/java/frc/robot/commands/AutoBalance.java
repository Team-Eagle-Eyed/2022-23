package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;


public class AutoBalance extends CommandBase{

    private final Swerve s_Swerve;
    private final PIDController forwardController;
    private final double kP = 0.2;
    private final double kI = 0;
    private final double kD = 0;
    private final double maxOutput = 0.6;
    private double currentPitch;
    private double previousPitch;
    private double tiltSpeed;
    private double output;

    public AutoBalance(Swerve subsystem) {
        this.s_Swerve = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Swerve);

        forwardController = new PIDController(kP, kI, kD);
        forwardController.setTolerance(3);
      }
    
    @Override
    public void initialize() {
        forwardController.setSetpoint(1);
    }

    @Override
    public void execute() {
        currentPitch = s_Swerve.getPitch();
        
        tiltSpeed = previousPitch - currentPitch;

        tiltSpeed = (tiltSpeed + previousPitch - currentPitch) * 0.4;

        if(Math.abs(tiltSpeed) > 0.2 || Math.abs(currentPitch) < 7) {
            forwardController.setP(0);
        } else {
            forwardController.setP(0.2);
        }

        SmartDashboard.putNumber("Tiltspeed", tiltSpeed);

        output = MathUtil.clamp(forwardController.calculate(currentPitch, 1), -maxOutput, maxOutput);
        SmartDashboard.putNumber("Output Power", output);
        SmartDashboard.putNumber("Current Pitch2", currentPitch);

        s_Swerve.drive(new Translation2d(-output, 0), 0, true, true);
        previousPitch = currentPitch;

        if(forwardController.atSetpoint()) {
            this.cancel();
        }
    }
}
