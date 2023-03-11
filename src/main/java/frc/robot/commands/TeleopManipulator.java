package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class TeleopManipulator extends CommandBase {

    private final Manipulator s_Manipulator;
    private final BooleanSupplier manipulatorForward;
    private final BooleanSupplier manipulatorReverse;
    private double manipulatorSpeed = 0;

    public TeleopManipulator(Manipulator subsystem, BooleanSupplier manipulatorForward, BooleanSupplier manipulatorReverse) {
        s_Manipulator = subsystem;
        this.manipulatorForward = manipulatorForward;
        this.manipulatorReverse = manipulatorReverse;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Manipulator);
      }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      s_Manipulator.configureMotors();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (manipulatorForward.getAsBoolean()) {
          manipulatorSpeed = 0.8;
        } else if (manipulatorReverse.getAsBoolean()) {
          manipulatorSpeed = -0.15;
        } else {
          manipulatorSpeed = 0;
        }
        s_Manipulator.run(manipulatorSpeed);
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
