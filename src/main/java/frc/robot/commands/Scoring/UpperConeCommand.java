// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class UpperConeCommand extends CommandBase {
  /** Creates a new HighConeCommand. */
  private final ExtendoSubsystem extendoSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ClawSubsystem clawSubsystem;

  public UpperConeCommand(ExtendoSubsystem eSubsystem, ArmSubsystem aSubsystem, ClawSubsystem cSubsystem) {

    extendoSubsystem = eSubsystem;
    armSubsystem = aSubsystem;
    clawSubsystem = cSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(eSubsystem);
    addRequirements(aSubsystem);
    addRequirements(cSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (armSubsystem.armUpperCone()) {
      if (extendoSubsystem.extendoUpperCone()) {
        // lawSubsystem.clawOpen();
      }
    }

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
