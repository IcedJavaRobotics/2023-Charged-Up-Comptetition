// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class checkGyroCommand extends CommandBase {
  /** Creates a new checkGyro. */
  private final DriveTrainSubsystem driveTrainSubsystem;
  public checkGyroCommand(DriveTrainSubsystem dSubsystem) {
    driveTrainSubsystem = dSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrainSubsystem.checkGyro();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
