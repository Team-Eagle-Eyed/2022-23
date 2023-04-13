package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class TeleopArm extends CommandBase {

    private final Arm s_Arm;
    private final DoubleSupplier POV;
    private final DoubleSupplier shoulderSpeed;

    public TeleopArm(Arm subsystem, DoubleSupplier POV, DoubleSupplier shoulderSpeed) {
        s_Arm = subsystem;
        this.POV = POV;
        this.shoulderSpeed = shoulderSpeed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Arm);
      }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (POV.getAsDouble() == 0) {
        s_Arm.telescopeArm(1);
      } else if(POV.getAsDouble() == 180) {
        s_Arm.telescopeArm(-1);
      } else {
        s_Arm.telescopeArm(-0.03); // feedforward
      }

      if(POV.getAsDouble() == 90) {
        s_Arm.rotateArm(1);
      } else if (POV.getAsDouble() == 270) {
        s_Arm.rotateArm(-1);
      } else {
        s_Arm.rotateArm(0);
      }
      if(-shoulderSpeed.getAsDouble() < 0) {
        s_Arm.runShoulder(-shoulderSpeed.getAsDouble() * 0.5);
      } else {
        s_Arm.runShoulder(-shoulderSpeed.getAsDouble());
      }
      s_Arm.putTemperatures();
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
