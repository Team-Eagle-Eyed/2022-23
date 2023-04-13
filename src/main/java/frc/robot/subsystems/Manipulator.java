//https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {

    private final CANSparkMax m_leftManipulatorWheel = new CANSparkMax(Constants.Manipulator.manipulatorLeftID, MotorType.kBrushless);
    private final CANSparkMax m_rightManipulatorWheel = new CANSparkMax(Constants.Manipulator.manupulatorRightID, MotorType.kBrushless);
    
    public Manipulator() {
        configureMotors();
    }

    public void run(double speed) {
        m_leftManipulatorWheel.set((MathUtil.applyDeadband(speed, 0.1)));
        m_rightManipulatorWheel.set((MathUtil.applyDeadband(speed, 0.1)));
    }

    public void configureMotors() {
        m_leftManipulatorWheel.setSmartCurrentLimit(14);
        m_rightManipulatorWheel.setSmartCurrentLimit(14);
        m_leftManipulatorWheel.burnFlash();
        m_rightManipulatorWheel.burnFlash();
    }
}
