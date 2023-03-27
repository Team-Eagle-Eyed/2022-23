// docs and pattern table: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
    private Spark blinkin = new Spark(0);

    public Lights() {}

    public void flashError() {
        blinkin.set(-0.11); // strobe, red or 0.61 for solid red
    }

    public void blue() {
        blinkin.set(0.85); // solid dark blue, or 0.87 for regular blue
    }

    public void custom(double pattern) {
        blinkin.set(pattern);
    }

    public void off() {
        blinkin.set(0); // hopefully this turns them off
    }

    public double getPattern() {
        return blinkin.get();
    }
}
