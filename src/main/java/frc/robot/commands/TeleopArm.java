package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TeleopArm extends CommandBase {

    private final Arm s_Arm;
    private final DoubleSupplier telescopeSpeed;
    private final DoubleSupplier shoulderSpeed;
    private final DoubleSupplier pivotSpeed;

    public TeleopArm(Arm subsystem, DoubleSupplier telescopeSpeed, DoubleSupplier shoulderSpeed, DoubleSupplier pivotSpeed) {
        s_Arm = subsystem;
        this.telescopeSpeed = telescopeSpeed;
        this.shoulderSpeed = shoulderSpeed;
        this.pivotSpeed = pivotSpeed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Arm);
      }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      s_Arm.configureMotors();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        s_Arm.telescopeArm(telescopeSpeed.getAsDouble());
        s_Arm.runShoulder(-shoulderSpeed.getAsDouble());
        s_Arm.rotateArm(MathUtil.applyDeadband(pivotSpeed.getAsDouble(), 0.1));
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
