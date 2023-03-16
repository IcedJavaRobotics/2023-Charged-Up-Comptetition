// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawOpenCommand extends CommandBase {
  /** Creates a new OpenClawCommand. */

  private final ClawSubsystem clawSubsystem;

  public ClawOpenCommand(ClawSubsystem subsystem) {

    clawSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clawSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    clawSubsystem.clawOpen();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    clawSubsystem.clawStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
