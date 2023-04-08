// docs and pattern table: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
    private Spark blinkin = new Spark(0);

    public Lights() {}

    public void custom(double pattern) {
        blinkin.set(pattern);
    }

    public double getPattern() {
        return blinkin.get();
    }
}
