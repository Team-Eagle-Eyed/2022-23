// https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final CANSparkMax m_armRotate = new CANSparkMax(Constants.Arm.armRotateID, MotorType.kBrushless);
    private final CANSparkMax m_armTelescope = new CANSparkMax(Constants.Arm.armTelescopeID, MotorType.kBrushless);
    private final CANSparkMax m_armShoulder = new CANSparkMax(Constants.Arm.armShoulderID, MotorType.kBrushless);
    
    public Arm() {}

    public void rotateArm(double speed) {
        m_armRotate.set(speed * 0.5);
    }

    public void telescopeArm(double speed) {
        m_armTelescope.set(speed);

    }

    public void runShoulder(double speed) {
        if(speed < 0) {
            m_armShoulder.set(speed * 0.5);
        } else {
            m_armShoulder.set(speed);
        }
    }

    public void configureMotors() {
        m_armShoulder.setOpenLoopRampRate(0.3);
        m_armShoulder.setSmartCurrentLimit(10);
        m_armShoulder.setInverted(true);
        m_armRotate.setOpenLoopRampRate(0.5);
        m_armRotate.setInverted(true);
        m_armTelescope.setSmartCurrentLimit(8);
        m_armTelescope.setInverted(true);
        m_armTelescope.setOpenLoopRampRate(0);

        m_armShoulder.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_armShoulder.setSoftLimit(SoftLimitDirection.kForward, 93);

        m_armTelescope.burnFlash();
        m_armRotate.burnFlash();
        m_armShoulder.burnFlash();
    }
}
