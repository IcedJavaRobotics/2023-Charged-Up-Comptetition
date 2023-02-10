// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExtendoSubsystem extendoSubsystem = new ExtendoSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  XboxController xboxController = new XboxController(Constants.CONTROLLER);
  Joystick flightStick = new Joystick(Constants.JOYSTICK);
  Joystick driverStation = new Joystick(Constants.DRIVER_STATION);


  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings(); {}
    
    driveTrainSubsystem.setDefaultCommand(
      new RunCommand(() -> driveTrainSubsystem.mecanumDrive(-getJoystickX(), getJoystickY(), 0.87 * -getJoystickTwist(), flightStick.getThrottle(), flightStick.getRawButton(1)), driveTrainSubsystem)
    );

  }

  private double deadZoneMod(double val) {      //Creates a range where the robot will not recieve input to prevent controller drift
    if (Math.abs(val) <= Constants.DEADZONE) {
      return 0;
    } else {
      return ((Math.abs(val) - 0.2) * 1.25) * (val/Math.abs(val));
    }
  }

  public double getJoystickX() {
    if ( flightStick != null ) {
      return deadZoneMod(flightStick.getX());
    } else {
      return 0;
    }
  }

  public double getJoystickY() {
    if ( flightStick != null ) {
      return deadZoneMod(flightStick.getY());
    } else {
      return 0;
    }
  }

  public double getJoystickTwist() {
    if ( flightStick != null ) {
      return deadZoneMod(flightStick.getTwist());
    } else {
      return 0;
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem,driveTrainSubsystem,clawSubsystem,armSubsystem,extendoSubsystem);
  }
}
