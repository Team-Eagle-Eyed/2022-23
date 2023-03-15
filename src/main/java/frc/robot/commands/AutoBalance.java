package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;


public class AutoBalance extends CommandBase {

    private final Swerve s_Swerve;
    private final PIDController pitchController;
    private final PIDController rollController;
    private final double kP = 0.2;
    private final double kI = 0;
    private final double kD = 0;
    private final double maxOutput = 0.6;
    private double currentPitch;
    private double previousPitch;
    private double pitchTiltSpeed;
    private double pitchOutput;
    
    private double passCount;

    public AutoBalance(Swerve subsystem) {
        this.s_Swerve = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Swerve);

        pitchController = new PIDController(kP, kI, kD);
        pitchController.setTolerance(3);
        rollController = new PIDController(kP, kI, kD);
        rollController.setTolerance(3);
      }
    
    @Override
    public void initialize() {
        pitchController.setSetpoint(1);
        rollController.setSetpoint(1);
    }

    @Override
    public void execute() {
        /* Pitch correction calculations */
        currentPitch = s_Swerve.getPitch();
        pitchTiltSpeed = previousPitch - currentPitch;
        pitchTiltSpeed = (pitchTiltSpeed + previousPitch - currentPitch) * 0.4;
        if(Math.abs(pitchTiltSpeed) > 0.2 || Math.abs(currentPitch) < 7) {
            pitchController.setP(0);
        } else {
            pitchController.setP(kP);
        }
        pitchOutput = MathUtil.clamp(pitchController.calculate(currentPitch, 1), -maxOutput, maxOutput);
        previousPitch = currentPitch;
        
        s_Swerve.drive(new Translation2d(-pitchOutput, 0), 0, false, true);
        if(pitchController.atSetpoint()/*  && rollController.atSetpoint() */) {
            passCount++;
        } else {
            passCount = 0;
        }
        if(passCount >= 50) {
            s_Swerve.drive(new Translation2d(0, 0.05), 0, true, true);
            this.cancel();
        }
    }
}
