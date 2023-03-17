// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinSubsystem;

public class LightsCubeCommand extends CommandBase {
  /** Creates a new LightsCubeCommand. */
  private final BlinkinSubsystem blinkinSubsystem;

  public LightsCubeCommand(BlinkinSubsystem bSubsystem) {

    blinkinSubsystem = bSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(bSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    blinkinSubsystem.colorCube();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    blinkinSubsystem.autoBlinkin();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
