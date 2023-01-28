// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GlobalVariablesSubsystem;

public class SetSlotGoal5Command extends CommandBase {
  /** Creates a new SetSlotGoal5Command. */

  private final GlobalVariablesSubsystem globalVariablesSubsystem;

  public SetSlotGoal5Command(GlobalVariablesSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    globalVariablesSubsystem = subsystem;
    addRequirements( globalVariablesSubsystem );

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    globalVariablesSubsystem.slotGoal5();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    globalVariablesSubsystem.slotGoalReset();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
