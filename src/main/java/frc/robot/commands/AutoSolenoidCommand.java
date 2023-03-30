// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;

public class AutoSolenoidCommand extends CommandBase {
  /** Creates a new AutoSolenoidCommand. */
  PneumaticSubsystem pneumaticSubsystem;

  public AutoSolenoidCommand(PneumaticSubsystem pSubsystem) {

    pneumaticSubsystem = pSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumaticSubsystem.forwardAutoSolenoid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pneumaticSubsystem.offAutoSolenoid();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
