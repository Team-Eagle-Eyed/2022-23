// https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final CANSparkMax m_armRotate = new CANSparkMax(Constants.Arm.armRotateID, MotorType.kBrushless);
    private final CANSparkMax m_armTelescope = new CANSparkMax(Constants.Arm.armTelescopeID, MotorType.kBrushless);
    private final CANSparkMax m_armShoulder = new CANSparkMax(Constants.Arm.armShoulderID, MotorType.kBrushless);
    private final CANCoder shoulderEncoder = new CANCoder(5, "*");
    public final RelativeEncoder integratedShoulderEncoder = m_armShoulder.getEncoder();
    private final AnalogPotentiometer telescopePot = new AnalogPotentiometer(0);
    
    public Arm() {
        shoulderEncoder.configSensorDirection(true);
    }

    public void rotateArm(double speed) {
        //m_armRotate.set(speed * 0.5); // disabled for now
    }

    public void telescopeArm(double speed) {
        m_armTelescope.set(speed);
        SmartDashboard.putNumber("ArmPosition", m_armTelescope.getEncoder().getPosition());
        SmartDashboard.putNumber("ArmPot", telescopePot.get());

    }

    public void runShoulder(double speed) {
        m_armShoulder.set(speed);
        SmartDashboard.putNumber("ShoulderPosition", shoulderEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("ShoulderIntPos", m_armShoulder.getEncoder().getPosition());
    }

    public void putTemperatures() {
        SmartDashboard.putNumber("m_armTelescope_temp", m_armTelescope.getMotorTemperature());
        SmartDashboard.putNumber("m_armShoulder_temp", m_armShoulder.getMotorTemperature());
        SmartDashboard.putNumber("m_armRotate_temp", m_armRotate.getMotorTemperature());
    }

    public void configureMotors() {
        m_armShoulder.setOpenLoopRampRate(0.3);
        m_armShoulder.setSmartCurrentLimit(15);
        m_armShoulder.setInverted(true);
        m_armShoulder.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_armShoulder.setSoftLimit(SoftLimitDirection.kForward, 93);
        m_armShoulder.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_armShoulder.setSoftLimit(SoftLimitDirection.kReverse, 0);
        m_armShoulder.setIdleMode(IdleMode.kBrake);

        m_armShoulder.getEncoder().setPosition((shoulderEncoder.getAbsolutePosition() - 184.75) * 0.822);

        m_armRotate.setOpenLoopRampRate(0.5);
        m_armRotate.setInverted(true);

        m_armTelescope.setSmartCurrentLimit(5);
        m_armTelescope.setInverted(true);
        m_armTelescope.setOpenLoopRampRate(0);
        m_armTelescope.setIdleMode(IdleMode.kBrake);

        m_armTelescope.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_armTelescope.setSoftLimit(SoftLimitDirection.kForward, 195); // replace 1200 with the number of rotations in a full extension
        m_armTelescope.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_armTelescope.setSoftLimit(SoftLimitDirection.kReverse, 23);
        m_armTelescope.getEncoder().setPosition((210 * telescopePot.get())); // replace 1200 with the number of rotations in a full extension, also the potentiometer might not get to the full 1 or 0
       

        m_armTelescope.burnFlash();
        m_armRotate.burnFlash();
        m_armShoulder.burnFlash();
    }
}
