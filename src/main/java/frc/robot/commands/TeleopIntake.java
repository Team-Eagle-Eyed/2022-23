package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TeleopIntake extends CommandBase {
    private final Intake s_Intake;
    private final DoubleSupplier intakeSup;
    private final BooleanSupplier turboFlail;
    private final BooleanSupplier objectPresent;

    public TeleopIntake(Intake subsystem, DoubleSupplier intakeSup, BooleanSupplier turboFlail, BooleanSupplier objectPresent) {
        s_Intake = subsystem;
        this.intakeSup = intakeSup;
        this.turboFlail = turboFlail;
        this.objectPresent = objectPresent;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(s_Intake);
      }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double intakeVal =
            MathUtil.applyDeadband(intakeSup.getAsDouble(), frc.robot.Constants.Intake.stickDeadband);
        if (turboFlail.getAsBoolean()) {
            if(intakeVal < 0) { //negative values outtake
                s_Intake.run(intakeVal); //out at full speed
            } else {
                s_Intake.run(intakeVal * 0.45); //in at limited speed
            }    
        } else {
            if(intakeVal < 0) { //negative values outtake
                s_Intake.run(intakeVal * 0.17); //out at low speed
            } else {
                s_Intake.run(intakeVal * 0.25 * (objectPresent.getAsBoolean() ? 0 : 1)
                ); //in at limited speed
            }
        }
        
    }
}
