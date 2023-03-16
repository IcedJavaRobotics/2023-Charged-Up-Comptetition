// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class ExtendCommand extends CommandBase {
  /** Creates a new ResetCommand. */
  private final ExtendoSubsystem extendoSubsystem;
  private final ClawSubsystem clawSubsystem;
  private double controllerY;

  public ExtendCommand(ExtendoSubsystem eSubsystem, ClawSubsystem cSubsystem, double xboxY) {
    extendoSubsystem = eSubsystem;
    clawSubsystem = cSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(eSubsystem);
    addRequirements(cSubsystem);
    controllerY = xboxY;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (clawSubsystem.clawBudge()) {
      extendoSubsystem.extendoJoystick(controllerY);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extendoSubsystem.extendoStop();
    clawSubsystem.clawStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
