// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class ResetCommand extends CommandBase {
  /** Creates a new ResetCommand. */
  private final ExtendoSubsystem extendoSubsystem;
  private final ClawSubsystem clawSubsystem;

  public ResetCommand(ExtendoSubsystem eSubsystem, ClawSubsystem cSubsystem) {
    extendoSubsystem = eSubsystem;
    clawSubsystem = cSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(eSubsystem);
    addRequirements(cSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (extendoSubsystem.extendoReturn()) {
      clawSubsystem.clawOpen();
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
