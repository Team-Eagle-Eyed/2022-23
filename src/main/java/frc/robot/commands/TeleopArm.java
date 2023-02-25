package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TeleopArm extends CommandBase {

    private final Arm s_Arm;
    private final DoubleSupplier telescopeSpeed;
    private final DoubleSupplier shoulderSpeed;
    private final DoubleSupplier pivotLeftSpeed;
    private final DoubleSupplier pivotRightSpeed;

    public TeleopArm(Arm subsystem, DoubleSupplier telescopeSpeed, DoubleSupplier shoulderSpeed, DoubleSupplier pivotLeftSpeed, DoubleSupplier pivotRightSpeed) {
        s_Arm = subsystem;
        this.telescopeSpeed = telescopeSpeed;
        this.shoulderSpeed = shoulderSpeed;
        this.pivotLeftSpeed = pivotLeftSpeed;
        this.pivotRightSpeed = pivotRightSpeed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Arm);
      }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Arm.telescopeArm(telescopeSpeed.getAsDouble());
        s_Arm.runShoulder(shoulderSpeed.getAsDouble());
        s_Arm.rotateArm(-(pivotRightSpeed.getAsDouble() - pivotLeftSpeed.getAsDouble()));
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
