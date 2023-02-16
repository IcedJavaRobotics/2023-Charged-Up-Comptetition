// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final ClawSubsystem clawSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ExtendoSubsystem extendoSubsystem;
  private final ExampleSubsystem exampleSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private int mode = 1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem, DriveTrainSubsystem msubsystem, ClawSubsystem csubsystem,
      ArmSubsystem asubsystem, ExtendoSubsystem esubsystem, LimelightSubsystem lsubsystem) {

    driveTrainSubsystem = msubsystem;
    clawSubsystem = csubsystem;
    armSubsystem = asubsystem;
    extendoSubsystem = esubsystem;
    exampleSubsystem = subsystem;
    limelightSubsystem = lsubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(msubsystem);
    addRequirements(csubsystem);
    addRequirements(asubsystem);
    addRequirements(esubsystem);
    addRequirements(subsystem);
    addRequirements(lsubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mode = exampleSubsystem.getMode();

    /**
     * If we wanted the drive train to be in brake mode during autonomous
     */
    // driveTrainSubsystem.frontLeftTalon.setNeutralMode(NeutralMode.Brake);
    // driveTrainSubsystem.backLeftTalon.setNeutralMode(NeutralMode.Brake);
    // driveTrainSubsystem.frontRightTalon.setNeutralMode(NeutralMode.Brake);
    // driveTrainSubsystem.backLeftTalon.setNeutralMode(NeutralMode.Brake);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    modeFunction(mode); // changes what its excecuting based on which mode its on

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
    // driveTrainSubsystem.frontLeftTalon.setNeutralMode(NeutralMode.Coast);
    // driveTrainSubsystem.backLeftTalon.setNeutralMode(NeutralMode.Coast);
    // driveTrainSubsystem.frontRightTalon.setNeutralMode(NeutralMode.Coast);
    // driveTrainSubsystem.backLeftTalon.setNeutralMode(NeutralMode.Coast);
    return false;
  }

  /**
   * <ul>
   * <li>mode 1: TaxiOutShort
   * <li>mode 2: TaxiOutLong
   * <li>mode 3: WIP
   * </ul>
   * 
   * @param Mode which mode you want,
   * 
   * 
   */
  public void modeFunction(int Mode) {
    // sees which mode you are on(check buttons folder)
    if (Mode == 1) {
      /*
       * first mode-
       * moves left, then runs high arm, then extends arm to upper,
       * then opens claw, then retracts arm,
       * then lowers arm and makes robot move taxiOutShort backwards out of the
       * community
       */
      if (!driveTrainSubsystem.moveLeft()) { // Checks if moveLeft is done

        if ((!armSubsystem.highArm()) && (!extendoSubsystem.extendoUpper())) {
          // runs high arm until done, then runs extendoUpper until done
          if (!clawSubsystem.clawOpen()) {// runs clawOpen until done, then does taxiOutShort
            if (!extendoSubsystem.extendoDefault()) { // retracts arm to default position
              armSubsystem.lowerArm();
              driveTrainSubsystem.taxiOutShort(); // makes robot move backwards out of the community
            }
          }

        }
      }

    } else if (Mode == 2) {
      /*
       * second mode-
       * moves left, then runs high arm, then extends arm to upper,
       * then opens claw, then retracts arm,
       * then lowers arm and makes robot move taxiOutLong backward out of the
       * community
       */
      if (!driveTrainSubsystem.moveLeft()) { // Checks if moveLeft is done

        if ((!armSubsystem.highArm()) && (!extendoSubsystem.extendoUpper())) {
          // runs high arm until done, then runs extendoUpper until done
          if (!clawSubsystem.clawOpen()) {// runs clawOpen until done, then does taxiOutShort
            if (!extendoSubsystem.extendoDefault()) { // retracts arm to default position
              armSubsystem.lowerArm();
              driveTrainSubsystem.taxiOutLong(); // makes robot move backwards out of the community
            }
          }

        }
      }

    } else if (Mode == 3) {
      // third mode-(insert what the mode does here)
      // TODO put third option here.

    } else {

      System.out.println("error 404: mode not found");

    }
  }
}
