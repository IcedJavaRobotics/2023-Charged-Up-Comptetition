// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class HighGridCommand extends CommandBase {
  
  private final ArmSubsystem armSubsystem;
  private final ExtendoSubsystem extendoSubsystem;

  public HighGridCommand(ArmSubsystem asubsystem, ExtendoSubsystem esubsystem) {
    armSubsystem = asubsystem;
    extendoSubsystem = esubsystem;

    addRequirements(asubsystem);
    addRequirements(esubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (armSubsystem.armController.atSetpoint() == true) {
    //   System.out.println("at set point");
    // } else {
    //   System.out.println("not");
    // }

    if (armSubsystem.armController.atSetpoint() == true) {
      extendoSubsystem.extendoHighGrid();
    } else {
      armSubsystem.armHighGrid();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopArm();
    extendoSubsystem.extendoStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
