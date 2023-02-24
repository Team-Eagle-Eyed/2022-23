// https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final CANSparkMax m_armRotate = new CANSparkMax(Constants.Arm.armRotateID, MotorType.kBrushless);
    private final CANSparkMax m_armTelescope = new CANSparkMax(Constants.Arm.armTelescopeID, MotorType.kBrushless);
    private final CANSparkMax m_armShoulder = new CANSparkMax(Constants.Arm.armShoulderID, MotorType.kBrushless);
    
    public Arm() {}

    public void rotateArm(double speed) {
        System.out.println("Setting rotate speed");
        m_armRotate.set(speed);
    }

    public void telescopeArm(double speed) {
        System.out.println("Setting telescope speed to " + speed);
        m_armTelescope.set(speed);
        System.out.println(m_armTelescope.get());

    }

    public void runShoulder(double speed) {
        m_armShoulder.set(speed);
    }
}
