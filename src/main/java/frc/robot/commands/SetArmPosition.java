package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArmPosition extends CommandBase {

    private final double kP = 0.1;
    private final double kI = 0;
    private final double kD = 0;
    private final double setpoint;

    private final Arm s_Arm;

    private final RelativeEncoder integratedShoulderEncoder;
    private final PIDController shoulderController;

    public SetArmPosition(Arm subsystem, double setpoint) {
        s_Arm = subsystem;
        this.setpoint = setpoint;
        shoulderController = new PIDController(kP, kI, kD);
        integratedShoulderEncoder = s_Arm.integratedShoulderEncoder;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Arm);
      }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      shoulderController.setTolerance(3);
      shoulderController.setSetpoint(setpoint);
      // s_Arm.configureMotors(); // probably don't need this
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      s_Arm.runShoulder(MathUtil.clamp(shoulderController.calculate(integratedShoulderEncoder.getPosition()), -1, 1));
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
