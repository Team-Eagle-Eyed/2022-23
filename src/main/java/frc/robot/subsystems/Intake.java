package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;


public class Intake extends SubsystemBase {

    private CANSparkMax m_leftIntakeSpinner = new CANSparkMax(Constants.Intake.leftIntakeSpinnerID, MotorType.kBrushless);
    private CANSparkMax m_rightIntakeSpinner = new CANSparkMax(Constants.Intake.rightIntakeSpinnerID, MotorType.kBrushless);

    public Intake() {}

    public void run(double speed) {
        m_leftIntakeSpinner.set(-speed);
        m_rightIntakeSpinner.set(speed);
    }

    public void configureMotors() {
        // nothing rn
    }
}
