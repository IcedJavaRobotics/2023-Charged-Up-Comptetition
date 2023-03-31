// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExtendoSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final ClawSubsystem clawSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ExtendoSubsystem extendoSubsystem;
  private final ExampleSubsystem exampleSubsystem;
  private final PneumaticSubsystem pneumaticSubsystem;
  private final BlinkinSubsystem blinkinSubsystem;
  private int mode = 1;
  private boolean firstStepDone;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem, DriveTrainSubsystem msubsystem, ClawSubsystem csubsystem,
      ArmSubsystem asubsystem, ExtendoSubsystem esubsystem, PneumaticSubsystem pSubsystem,
      BlinkinSubsystem bSubsystem) {

    driveTrainSubsystem = msubsystem;
    clawSubsystem = csubsystem;
    armSubsystem = asubsystem;
    extendoSubsystem = esubsystem;
    exampleSubsystem = subsystem;
    pneumaticSubsystem = pSubsystem;
    blinkinSubsystem = bSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(msubsystem);
    addRequirements(csubsystem);
    addRequirements(asubsystem);
    addRequirements(esubsystem);
    addRequirements(subsystem);
    addRequirements(pSubsystem);
    addRequirements(bSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mode = exampleSubsystem.getMode();
    blinkinSubsystem.autoBlinkin();
    driveTrainSubsystem.zeroEncoder();
    pneumaticSubsystem.forwardAutoSolenoid();
    pneumaticSubsystem.forwardDriveSolenoid();
    firstStepDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (firstStepDone == false) {
      firstStepDone = driveTrainSubsystem.autoTaxi();
    } else if (firstStepDone == true) {
      driveTrainSubsystem.autoCharging();
    }

    // modeFunction(exampleSubsystem.getMode()); // changes what its excecuting
    // based on which mode its on

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    firstStepDone = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
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
   *
   *             public void modeFunction(int Mode) {
   *             // sees which mode you are on(check buttons folder)
   *             if (Mode == 1) {
   * 
   *             driveTrainSubsystem.taxiOutLong();
   * 
   *             } else if (Mode == 2) {
   * 
   *             } else if (Mode == 3) {
   * 
   *             } else {
   * 
   *             System.out.println("error 404: mode not found");
   * 
   *             }
   *             }
   */
}
