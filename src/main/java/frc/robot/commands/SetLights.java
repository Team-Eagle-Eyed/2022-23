package frc.robot.commands;

import java.util.function.BooleanSupplier;

//import edu.wpi.first.wpilibj.boolean;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.lightPatterns;
import frc.robot.subsystems.Lights;

public class SetLights extends Command {
    private final Lights s_Lights;
    private final double pattern;
    private final BooleanSupplier objectSensor;

    public SetLights(Lights subsystem, double pattern, BooleanSupplier objectSensor) {
        s_Lights = subsystem;
        this.pattern = pattern;
        this.objectSensor = objectSensor;
        addRequirements(s_Lights);
    }

    public void execute() {
        if(objectSensor.getAsBoolean()) {
            s_Lights.custom(lightPatterns.green);
        } else {
            s_Lights.custom(pattern); // strobe, red or 0.61 for solid red
        }
    }
}
