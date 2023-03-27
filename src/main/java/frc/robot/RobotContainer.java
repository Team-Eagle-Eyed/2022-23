// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.balanceAutoOutside;
import frc.robot.autos.dummyOutside;
import frc.robot.autos.twoCube;
import frc.robot.autos.dummyInside;
import frc.robot.autos.balanceAutoCenter;
import frc.robot.autos.balanceAutoCharge;
import frc.robot.autos.balanceAutoInside;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.CenterTag;
import frc.robot.commands.SetArmPosition;
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
  private final GenericHID auxiliary = new GenericHID(2);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int speedAxis = XboxController.Axis.kRightTrigger.value;

  /* Arm Controls */
  private final int shoulderAxis = XboxController.Axis.kRightY.value;

  /* Manipulator Controls */
  private final int manipulatorForward = XboxController.Button.kX.value;
  private final int manipulatorReverse = XboxController.Button.kB.value;

  /* Intake Controls */
  private final int intakeAxis = XboxController.Axis.kLeftY.value;
  private final int turboFlail = XboxController.Axis.kRightTrigger.value;

  /* Driver Buttons */
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton zeroGyro2 = 
      new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton zeroGyro3 = 
      new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton resetWheels = 
      new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton autoBalance = 
      new JoystickButton(driver, XboxController.Button.kY.value);

  /* Operator Controls */
  private final JoystickButton setArmMid = 
      new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton setArmHigh = 
      new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton stowArm = 
      new JoystickButton(operator, XboxController.Button.kBack.value);

  /* Auxiliary Controls */
  private final JoystickButton estop = 
      new JoystickButton(auxiliary, 0);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Arm s_Arm = new Arm();
  private final Manipulator s_Manipulator = new Manipulator();
  private final Intake s_Intake = new Intake();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_chooser.setDefaultOption("Balance Auto Inside", new balanceAutoInside(s_Swerve, s_Intake));
    m_chooser.addOption("Balance Auto Outside", new balanceAutoOutside(s_Swerve, s_Intake));
    m_chooser.addOption("Balance Auto Center", new balanceAutoCenter(s_Swerve, s_Intake));
    m_chooser.addOption("Balance Auto Charge", new balanceAutoCharge(s_Swerve, s_Intake));
    m_chooser.addOption("Dummy Outside", new dummyOutside(s_Swerve, s_Intake));
    m_chooser.addOption("Dummy Inside", new dummyInside(s_Swerve, s_Intake));
    m_chooser.addOption("Two Cube Auto", new twoCube(s_Swerve, s_Intake));
    m_chooser.addOption("Center Tag Auto", new CenterTag(s_Swerve));
    m_chooser.addOption("Nothing", new InstantCommand());
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putNumber("SpeedLimit", 1);


    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
            () -> -driver.getRawAxis(strafeAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
            () -> -driver.getRawAxis(rotationAxis) * SmartDashboard.getNumber("SpeedLimit", 1) * 0.60,
            () -> robotCentric.getAsBoolean()));
    
    s_Arm.setDefaultCommand(
        new TeleopArm(s_Arm,
        () -> operator.getPOV(), //operator::getPOV,
        () -> operator.getRawAxis(shoulderAxis))
    );

    s_Manipulator.setDefaultCommand(
        new TeleopManipulator(s_Manipulator,
        () -> operator.getRawButton(manipulatorForward),
        () -> operator.getRawButton(manipulatorReverse))
    );

    s_Intake.setDefaultCommand(
        new TeleopIntake(s_Intake,
        () -> operator.getRawAxis(intakeAxis),
        () -> operator.getRawAxis(turboFlail) > 0.5 ? true : false) //if trigger is more than half pressed, turbo is enabled
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
    zeroGyro2.and(zeroGyro3).onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(0)));
    autoBalance.whileTrue(new AutoBalance(s_Swerve));
    resetWheels.onTrue(new InstantCommand(() -> s_Swerve.resetWheelsToAbsolute()));
    setArmMid.whileTrue(new SetArmPosition(s_Arm, 64));
    setArmHigh.whileTrue(new SetArmPosition(s_Arm, 78));
    stowArm.whileTrue(new SetArmPosition(s_Arm, 0));
    estop.whileTrue(new RepeatCommand(new InstantCommand(() -> {
        s_Arm.rotateArm(0);
        s_Arm.telescopeArm(0);
        s_Arm.runShoulder(0);
        s_Intake.run(0);
        s_Manipulator.run(0);
        s_Swerve.drive(new Translation2d(), 0, false, false);
    }, s_Arm, s_Intake, s_Manipulator, s_Swerve)));
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
