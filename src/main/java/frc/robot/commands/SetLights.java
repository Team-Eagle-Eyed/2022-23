package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class SetLights extends CommandBase {
    private final Lights s_Lights;
    private final double pattern;

    public SetLights(Lights subsystem, double pattern) {
        s_Lights = subsystem;
        this.pattern = pattern;
        addRequirements(s_Lights);
    }

    public void execute() {
        s_Lights.custom(pattern); // strobe, red or 0.61 for solid red
    }
}
