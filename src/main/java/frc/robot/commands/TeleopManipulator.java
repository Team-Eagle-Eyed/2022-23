package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class TeleopManipulator extends CommandBase {

    private final Manipulator s_Manipulator;
    private final boolean manipulatorForward;
    private final boolean manipulatorReverse;
    private double manipulatorSpeed = 0;

    public TeleopManipulator(Manipulator subsystem, boolean forward, boolean reverse) {
        s_Manipulator = subsystem;
        manipulatorForward = forward;
        manipulatorReverse = reverse;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Manipulator);
      }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (manipulatorForward) {
          manipulatorSpeed = 1;
        } else if (manipulatorReverse) {
          manipulatorSpeed = -1;
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
