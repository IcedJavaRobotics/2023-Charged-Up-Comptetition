// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.buttons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroCommand extends InstantCommand {

  private final ArmSubsystem armsubsystem;

  public ZeroCommand(ArmSubsystem asubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    armsubsystem = asubsystem;
    addRequirements(asubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // put the code in here!
    armsubsystem.zeroEncoder();
    System.out.println("Zeroing.....");
  }
}
