// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.lightPatterns;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.CenterTag;
import frc.robot.commands.SetLights;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

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

  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Intake s_Intake = new Intake();
  private final Lights s_Lights = new Lights();
  private final DigitalInput objectSensor = new DigitalInput(0);

  private final CenterTag c_centerTag = new CenterTag(s_Swerve, s_Lights);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putNumber("SpeedLimit", 1);
    autoChooser.addOption("Center Tag Auto", c_centerTag);


    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
            () -> -driver.getRawAxis(strafeAxis) * driver.getRawAxis(speedAxis) * SmartDashboard.getNumber("SpeedLimit", 1),
            () -> -driver.getRawAxis(rotationAxis) * SmartDashboard.getNumber("SpeedLimit", 1) * 0.60,
            () -> robotCentric.getAsBoolean()));

    s_Intake.setDefaultCommand(
        new TeleopIntake(s_Intake,
        () -> operator.getRawAxis(intakeAxis),
        () -> operator.getRawAxis(turboFlail) > 0.5 ? true : false,
        objectSensor::get) //if trigger is more than half pressed, turbo is enabled
    );

    s_Lights.setDefaultCommand(
        new SetLights(s_Lights, lightPatterns.blue, objectSensor::get)
    );


    NamedCommands.registerCommand("Center Tag", c_centerTag);

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
    //estop.whileTrue(new InstantCommand(() -> System.exit(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Will run in autonomous
    return autoChooser.getSelected();
  }
}
