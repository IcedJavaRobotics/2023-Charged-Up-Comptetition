// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PnuematicWheelsCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class RaiseWheelsCommand extends CommandBase {
  /** Creates a new RaiseWheelsCommand. */
  private DriveTrainSubsystem driveTrainSubsystem;
  private PneumaticSubsystem pneumaticSubsystem;

  public RaiseWheelsCommand(DriveTrainSubsystem dSubsystem, PneumaticSubsystem pSubsystem) {
    pneumaticSubsystem = pSubsystem;
    driveTrainSubsystem = dSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pSubsystem);
    addRequirements(dSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneumaticSubsystem.reverseDriveSolenoid();
    driveTrainSubsystem.wheelsRaised = true;
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
