// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroArmCommand extends CommandBase {
  /** Creates a new ZeroArmCommand. */
  private final ArmSubsystem armsubsystem;

  public ZeroArmCommand(ArmSubsystem subsystem) {
    armsubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armsubsystem.zeroEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
