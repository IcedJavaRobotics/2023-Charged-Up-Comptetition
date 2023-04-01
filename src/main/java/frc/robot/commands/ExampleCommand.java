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
import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrainSubsystem driveTrainSubsystem;
  private final ExampleSubsystem exampleSubsystem;
  private final PneumaticSubsystem pneumaticSubsystem;
  private final BlinkinSubsystem blinkinSubsystem;
  private int mode = 1;
  private boolean firstStepDone;
  private boolean secondStepDone;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem, DriveTrainSubsystem msubsystem, ClawSubsystem csubsystem,
      ArmSubsystem asubsystem, ExtendoSubsystem esubsystem, PneumaticSubsystem pSubsystem,
      BlinkinSubsystem bSubsystem) {

    driveTrainSubsystem = msubsystem;
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
    secondStepDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Auto 1

    // Shoots cube high then balances on charging station
    // Needs to be placed slightly off center and lined up with high cube score 
    if(Timer.getMatchTime() < 14) {

      if (firstStepDone == false) {
        firstStepDone = driveTrainSubsystem.autoTaxi();
      } else if (firstStepDone == true) {
        driveTrainSubsystem.autoCharging();
        pneumaticSubsystem.reverseAutoSolenoid();
      }

    }


    // Auto 2

    // Shoots cube high and taxis
    // Needs to be placed on side with out bump and lined up with cube score
    // if(Timer.getMatchTime() < 14) {

    //   driveTrainSubsystem.autoMove(0.5, 100);
    //   pneumaticSubsystem.reverseAutoSolenoid();
    // }


    // Auto 3

    // Only shoots cube high
    // Placed on side with bump and aligned with cube score

    // if (Timer.getMatchTime()< 14) {
    //   pneumaticSubsystem.reverseAutoSolenoid();
    // }


    // Auto 4

    // Hopefully shoots high cube, then taxis, and then balances

    // if(Timer.getMatchTime() < 14) {

    //   if (firstStepDone == false) {

    //     firstStepDone = driveTrainSubsystem.autoTaxi(0.5, 280);

    //   } else if (firstStepDone == true && secondStepDone == false) {

    //     secondStepDone = driveTrainSubsystem.autoTaxi2(-0.5,100);
    //     pneumaticSubsystem.reverseAutoSolenoid();

    //   } else if (firstStepDone == true && secondStepDone == true) {

    //     driveTrainSubsystem.autoCharging();
    //   }  
    // }



    // This switch statement chooses which auto to run 
    // Look at AUTO_MODE in constants for more info


    // switch(Constants.AUTO_MODE) {

    //   case 1: 

    //     // Shoots cube high then balances on charging station
    //     // Needs to be placed slightly off center and lined up with high cube score 
    //     if(Timer.getMatchTime() < 14) {

    //       if (firstStepDone == false) {
    //         firstStepDone = driveTrainSubsystem.autoTaxi();
    //       } else if (firstStepDone == true) {
    //         driveTrainSubsystem.autoCharging();
    //         pneumaticSubsystem.reverseAutoSolenoid();
    //       }

    //     }

    //     break;

    //   case 2:

    //     // Shoots cube high and taxis
    //     // Needs to be placed on side with out bump and lined up with cube score
    //     if(Timer.getMatchTime() < 14) {

    //       driveTrainSubsystem.autoMoveNoTank(0.5, 100);
    //       pneumaticSubsystem.reverseAutoSolenoid();
    //     }

    //     break;

    //   case 3:

    //     // Only shoots cube high
    //     // Placed on side with bump and aligned with cube score
    //     pneumaticSubsystem.reverseAutoSolenoid();

    //     break;

    //   case 4:

    //     // Hopefully shoots high cube, then taxis, and then balances
    //     if(Timer.getMatchTime() < 14) {

    //       if (firstStepDone == false) {

    //         firstStepDone = driveTrainSubsystem.autoTaxi(0.5, 280);

    //       } else if (firstStepDone == true && secondStepDone == false) {

    //         secondStepDone = driveTrainSubsystem.autoTaxi(-0.5,100);
    //         pneumaticSubsystem.reverseAutoSolenoid();

    //       } else if (firstStepDone == true && secondStepDone == true) {

    //         driveTrainSubsystem.autoCharging();
    //       }  
    //     }

    //     break;
    // }

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
