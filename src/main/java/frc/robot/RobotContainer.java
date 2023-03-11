// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.balanceAuto;
import frc.robot.autos.exampleAuto;
import frc.robot.autos.visionAuto;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopManipulator;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int speedAxis = XboxController.Axis.kRightTrigger.value;

  /* Arm Controls */
  private final int telescopeAxis = XboxController.Axis.kLeftY.value;
  private final int shoulderAxis = XboxController.Axis.kRightY.value;
  private final int armPivotAxis = XboxController.Axis.kLeftX.value;

  /* Manipulator Controls */
  private final int manipulatorForward = XboxController.Button.kX.value;
  private final int manipulatorReverse = XboxController.Button.kB.value;

  /* Intake Controls */
  private final int intakeAxis = XboxController.Axis.kRightY.value;


  /* Driver Buttons */
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton zeroGyro2 = 
      new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton zeroGyro3 = 
      new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton zeroGyro4 = 
      new JoystickButton(driver, XboxController.Button.kB.value);

  private final JoystickButton autoBalance = 
      new JoystickButton(driver, XboxController.Button.kY.value);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Arm s_Arm = new Arm();
  private final Manipulator s_Manipulator = new Manipulator();
  private final Intake s_Intake = new Intake();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_chooser.setDefaultOption("Balance Auto", new balanceAuto(s_Swerve));
    m_chooser.addOption("Vision Auto", new visionAuto(s_Swerve));
    m_chooser.addOption("Example Auto", new exampleAuto(s_Swerve));
    SmartDashboard.putData(m_chooser);


    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
            () -> -driver.getRawAxis(strafeAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
            () -> -driver.getRawAxis(rotationAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
            () -> robotCentric.getAsBoolean()));
    
    s_Arm.setDefaultCommand(
        new TeleopArm(s_Arm,
        () -> operator.getRawAxis(telescopeAxis),
        () -> operator.getRawAxis(shoulderAxis),
        () -> operator.getRawAxis(armPivotAxis))
    );

    s_Manipulator.setDefaultCommand(
        new TeleopManipulator(s_Manipulator,
        () -> operator.getRawButton(manipulatorForward),
        () -> operator.getRawButton(manipulatorReverse))
    );

    s_Intake.setDefaultCommand(
        new TeleopIntake(s_Intake,
        () -> driver.getRawAxis(intakeAxis))
    );

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro2.and(zeroGyro3).and(zeroGyro4).onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    autoBalance.onTrue(new AutoBalance(s_Swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Will run in autonomous
    return m_chooser.getSelected();
  }
}
