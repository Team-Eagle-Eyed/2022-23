package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TeleopArm extends CommandBase {

    private final Arm s_Arm;
    private final double telescopeSpeed;
    private final double shoulderSpeed;

    public TeleopArm(Arm subsystem, double telSpeed, double shouldSpeed) {
        s_Arm = subsystem;
        telescopeSpeed = telSpeed;
        shoulderSpeed = shouldSpeed;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Arm);
      }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Arm.telescopeArm(telescopeSpeed);
        s_Arm.runShoulder(shoulderSpeed);
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
