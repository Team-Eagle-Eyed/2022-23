//https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {

    private final CANSparkMax m_leftManipulatorWheel = new CANSparkMax(Constants.Manipulator.manipulatorLeftID, MotorType.kBrushless);
    private final CANSparkMax m_rightManipulatorWheel = new CANSparkMax(Constants.Manipulator.manupulatorRightID, MotorType.kBrushless);
    
    public Manipulator() {}

    public void run(double speed) {
        m_leftManipulatorWheel.set(-speed);
        m_rightManipulatorWheel.set(-speed);
    }
}
