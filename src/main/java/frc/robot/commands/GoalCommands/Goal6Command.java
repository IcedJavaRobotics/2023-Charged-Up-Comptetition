// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GoalCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

public class Goal6Command extends CommandBase {
  /** Creates a new Goal1Command. */
  private final DriveTrainSubsystem m_subsystem;
  private final ArmSubsystem a_subsystem;
  private final ClawSubsystem c_subsystem;
  private final ExtendoSubsystem e_subsystem;

  public Goal6Command(DriveTrainSubsystem msubsystem, ArmSubsystem asubsystem, ClawSubsystem csubsystem, ExtendoSubsystem esubsystem) {
    
    m_subsystem = msubsystem;
    a_subsystem = asubsystem;
    c_subsystem = csubsystem;
    e_subsystem = esubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(msubsystem);
    addRequirements(asubsystem);
    addRequirements(csubsystem);
    addRequirements(esubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_subsystem.moveLeft();
    
    if(m_subsystem.moveLeft() == false) {       //Checks if moveLeft is done

      e_subsystem.extendoMiddle();
      a_subsystem.middleArm();

      if((a_subsystem.middleArm() == false) && (e_subsystem.extendoMiddle() == false) ) {    // Checks if extendoUpper and highArm are done

        c_subsystem.clawOpen();

      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    c_subsystem.clawStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
