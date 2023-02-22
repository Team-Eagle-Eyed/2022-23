// https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final CANSparkMax m_armRotate = new CANSparkMax(Constants.Arm.armRotateID, MotorType.kBrushless);
    private final CANSparkMax m_armTelescope = new CANSparkMax(Constants.Arm.armTelescopeID, MotorType.kBrushless);
    private final CANSparkMax m_armShoulder = new CANSparkMax(Constants.Arm.armShoulderID, MotorType.kBrushless);
    
    public Arm() {}

    public CommandBase rotateArm(double speed) {
        return runOnce(
            () -> {
                m_armRotate.set(speed);
            });
    }

    public CommandBase telescopeArm(double speed) {
        return runOnce(
            () -> {
                m_armTelescope.set(speed);
            });
    }

    public CommandBase runShoulder(double speed) {
        return runOnce(
            () -> {
                m_armShoulder.set(speed);
            });
    }
}
