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
        m_armRotate.set(speed);
    }

    public void telescopeArm(double speed) {
        m_armTelescope.set(speed);

    }

    public void runShoulder(double speed) {
        m_armShoulder.set(speed);
    }

    public void configureMotors() {
        m_armShoulder.setOpenLoopRampRate(0.25);
        m_armTelescope.setSmartCurrentLimit(3);
        m_armTelescope.burnFlash();
        m_armShoulder.burnFlash();
    }
}
