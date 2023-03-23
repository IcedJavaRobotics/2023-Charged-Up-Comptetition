// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PneumaticWheelsSubsystem;
import frc.robot.commands.PnuematicWheelsCommands.DropWheelsCommand;
import frc.robot.commands.PnuematicWheelsCommands.RaiseWheelsCommand;
import frc.robot.commands.Scoring.MiddleConeCommand;
import frc.robot.commands.Scoring.MiddleCubeCommand;
import frc.robot.commands.Scoring.UpperConeCommand;
import frc.robot.commands.Scoring.UpperCubeCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ArmCommands.ClawCloseCommand;
import frc.robot.commands.ArmCommands.ClawOpenCommand;
import frc.robot.commands.ArmCommands.HighGridCommand;
import frc.robot.commands.ArmCommands.MidGridCommand;
import frc.robot.commands.ArmCommands.PickupArmCommand;
import frc.robot.commands.ArmCommands.ResetCommand;
import frc.robot.commands.ArmCommands.TuckArmCommand;
import frc.robot.commands.ArmCommands.ZeroArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Lights.LightsConeCommand;
import frc.robot.commands.Lights.LightsCubeCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  DigitalInput rightLimit = new DigitalInput(Constants.RIGHT_CLAW_LIMIT);
  DigitalInput leftLimit = new DigitalInput(Constants.LEFT_CLAW_LIMIT);
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExtendoSubsystem extendoSubsystem;
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ClawSubsystem clawSubsystem;
  private final PneumaticWheelsSubsystem pneumaticWheelsSubsystem = new PneumaticWheelsSubsystem();
  private final BlinkinSubsystem blinkinSubsystem = new BlinkinSubsystem();

  XboxController xboxController = new XboxController(Constants.CONTROLLER);
  Joystick flightStick = new Joystick(Constants.JOYSTICK);
  Joystick driverStation = new Joystick(Constants.DRIVER_STATION);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    clawSubsystem = new ClawSubsystem(rightLimit, leftLimit);
    extendoSubsystem = new ExtendoSubsystem(rightLimit, leftLimit);

    new JoystickButton(driverStation, 2)
        .whileTrue(new LightsConeCommand(blinkinSubsystem));

    new JoystickButton(driverStation, 3)
        .whileTrue(new LightsCubeCommand(blinkinSubsystem));

    new JoystickButton(driverStation, 1)
        .whileTrue(new TuckArmCommand(armSubsystem, extendoSubsystem));

    new JoystickButton(driverStation, 9)
        .whileTrue(new PickupArmCommand(armSubsystem, extendoSubsystem));

    new JoystickButton(driverStation, 6)
        .whileTrue(new MidGridCommand(armSubsystem, extendoSubsystem));

    new JoystickButton(driverStation, 8)
        .whileTrue(new HighGridCommand(armSubsystem, extendoSubsystem));
        
        
    // new JoystickButton(driverStation, 6)
    //     .whileTrue(new MiddleConeCommand(extendoSubsystem, armSubsystem, clawSubsystem));

    // new JoystickButton(driverStation, 8)
    //     .whileTrue(new MiddleCubeCommand(extendoSubsystem, armSubsystem, clawSubsystem));

    // new JoystickButton(driverStation, 1)
    //     .whileTrue(new UpperConeCommand(extendoSubsystem, armSubsystem, clawSubsystem));

    // new JoystickButton(driverStation, 9)
    //     .whileTrue(new UpperCubeCommand(extendoSubsystem, armSubsystem, clawSubsystem));

    new JoystickButton(xboxController, Constants.LEFT_TRIGGER)
        .whileTrue(new ClawCloseCommand(clawSubsystem, Constants.FAST_CLAW));
    new JoystickButton(xboxController, Constants.RIGHT_TRIGGER)
        .whileTrue(new ClawOpenCommand(clawSubsystem, Constants.FAST_CLAW));

    new JoystickButton(xboxController, Constants.LEFT_BUMPER)
	      .whileTrue(new ClawCloseCommand(clawSubsystem, Constants.SLOW_CLAW));

    new JoystickButton(xboxController, Constants.RIGHT_BUMPER)
	      .whileTrue(new ClawOpenCommand(clawSubsystem, Constants.SLOW_CLAW));

    new JoystickButton(xboxController, 1)
        .whileTrue(new ZeroArmCommand(armSubsystem));

    new JoystickButton(xboxController, 3)
        .whileTrue(new ResetCommand(extendoSubsystem, clawSubsystem));

    driveTrainSubsystem.setDefaultCommand(
        new RunCommand(() -> driveTrainSubsystem.mecanumDrive(getJoystickX(), -getJoystickY(),
            0.78 * getJoystickTwist(), flightStick.getThrottle()), driveTrainSubsystem));

    armSubsystem.setDefaultCommand(
        new RunCommand(() -> armSubsystem.armJoystick(xboxController.getLeftY()), armSubsystem));

    extendoSubsystem.setDefaultCommand(
        new RunCommand(() -> extendoSubsystem.extendoJoystick(-xboxController.getRightTriggerAxis()),
            extendoSubsystem));

    new JoystickButton(flightStick, 1)
        .whileTrue(new RaiseWheelsCommand(driveTrainSubsystem, pneumaticWheelsSubsystem));

    new JoystickButton(flightStick, 2)
        .whileTrue(new DropWheelsCommand(driveTrainSubsystem, pneumaticWheelsSubsystem));

  }

  private double deadZoneMod(double val) { // Creates a range where the robot will not recieve input to prevent
                                           // controller drift
    if (Math.abs(val) <= Constants.DEADZONE) {
      return 0;
    } else {
      return ((Math.abs(val) - 0.2) * 1.25) * (val / Math.abs(val));
    }
  }

  public double getJoystickX() {
    if (flightStick != null) {
      return deadZoneMod(flightStick.getX());
    } else {
      return 0;
    }
  }

  public double getJoystickY() {
    if (flightStick != null) {
      return deadZoneMod(flightStick.getY());
    } else {
      return 0;
    }
  }

  public double getJoystickTwist() {
    if (flightStick != null) {
      return deadZoneMod(flightStick.getTwist());
    } else {
      return 0;
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
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
    return Autos.exampleAuto(m_exampleSubsystem, driveTrainSubsystem, clawSubsystem, armSubsystem, extendoSubsystem,
        pneumaticWheelsSubsystem, blinkinSubsystem);
  }
}
