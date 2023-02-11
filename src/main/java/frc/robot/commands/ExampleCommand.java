// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrainSubsystem m_subsystem;
  private final ClawSubsystem c_Subsystem;
  private final ArmSubsystem a_Subsystem;
  private final ExtendoSubsystem e_Subsystem;
  private final ExampleSubsystem Subsystem;
  private int mode = 1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem, DriveTrainSubsystem msubsystem, ClawSubsystem csubsystem,
      ArmSubsystem asubsystem, ExtendoSubsystem esubsystem) {

    m_subsystem = msubsystem;
    c_Subsystem = csubsystem;
    a_Subsystem = asubsystem;
    e_Subsystem = esubsystem;
    Subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(msubsystem);
    addRequirements(csubsystem);
    addRequirements(asubsystem);
    addRequirements(esubsystem);
    addRequirements(subsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mode = Subsystem.getMode();

    /**
     * If we wanted the drive train to be in brake mode during autonomous
     */
    // m_subsystem.frontLeftTalon.setNeutralMode(NeutralMode.Brake);
    // m_subsystem.backLeftTalon.setNeutralMode(NeutralMode.Brake);
    // m_subsystem.frontRightTalon.setNeutralMode(NeutralMode.Brake);
    // m_subsystem.backLeftTalon.setNeutralMode(NeutralMode.Brake);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (mode) {
      case 1:
        m_subsystem.moveLeft();
        if (m_subsystem.moveLeft() == false) { // Checks if moveLeft is done

          a_Subsystem.highArm();
          e_Subsystem.extendoUpper();

          if ((a_Subsystem.highArm() == false) && (e_Subsystem.extendoUpper() == false)) {

            c_Subsystem.clawOpen();

          }
        }
        break;

      case 2:
        m_subsystem.moveLeft();
        if (m_subsystem.moveLeft() == false) { // Checks if moveLeft is done

          a_Subsystem.highArm();
          e_Subsystem.extendoUpper();

          if ((a_Subsystem.highArm() == false) && (e_Subsystem.extendoUpper() == false)) {

            c_Subsystem.clawOpen();

          }
        }
        break;
      case 3:
        m_subsystem.moveLeft();
        if (m_subsystem.moveLeft() == false) { // Checks if moveLeft is done

          a_Subsystem.highArm();
          e_Subsystem.extendoUpper();

          if ((a_Subsystem.highArm() == false) && (e_Subsystem.extendoUpper() == false)) {

            c_Subsystem.clawOpen();

          }
        }
        break;
      default:
        System.out.println("mode isnt a mode set, mode is: " + mode);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    /**
     * If we wanted the drive train to be in brake mode during autonomous
     */
    // m_subsystem.frontLeftTalon.setNeutralMode(NeutralMode.Coast);
    // m_subsystem.backLeftTalon.setNeutralMode(NeutralMode.Coast);
    // m_subsystem.frontRightTalon.setNeutralMode(NeutralMode.Coast);
    // m_subsystem.backLeftTalon.setNeutralMode(NeutralMode.Coast);
    return false;
  }
}
