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
    private double currentRoll;
    private double previousPitch;
    private double previousRoll;
    private double pitchTiltSpeed;
    private double rollTiltSpeed;
    private double pitchOutput;
    private double rollOutput;
    
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

        /* Roll correction calculations */
        currentRoll = s_Swerve.getRoll();
        rollTiltSpeed = previousRoll - currentRoll;
        rollTiltSpeed = (rollTiltSpeed + previousRoll - currentRoll) * 0.4;
        if(Math.abs(rollTiltSpeed) > 0.2 || Math.abs(currentRoll) < 7) {
            rollController.setP(0);
        } else {
            rollController.setP(kP);
        }
        rollOutput = MathUtil.clamp(rollController.calculate(currentRoll, 1), -maxOutput, maxOutput);
        previousRoll = currentRoll;
        
        s_Swerve.drive(new Translation2d(-pitchOutput, -rollOutput), 0, false, true);
        if(pitchController.atSetpoint() && rollController.atSetpoint()) {
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
