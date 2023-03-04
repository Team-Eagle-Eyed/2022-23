package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends CommandBase {
    private final Intake s_Intake;
    private final DoubleSupplier intakeSup;

    public TeleopIntake(Intake subsystem, DoubleSupplier intakeSup) {
        s_Intake = subsystem;
        this.intakeSup = intakeSup;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Intake);
      }

    @Override
    public void initialize() {
        s_Intake.configureMotors();
    }

    @Override
    public void execute() {
        double intakeVal =
            MathUtil.applyDeadband(intakeSup.getAsDouble(), frc.robot.Constants.Intake.stickDeadband) * frc.robot.Constants.Intake.maxIntakeSpeed;

        s_Intake.run(intakeVal);
    }
}
