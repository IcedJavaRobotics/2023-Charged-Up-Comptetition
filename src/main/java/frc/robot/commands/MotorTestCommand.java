// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class MotorTestCommand extends CommandBase {
  /** Creates a new MotorTestCommand. */
  private final DriveTrainSubsystem drivetrainSubsystem;
  /**
   * 
   * @param dsubsystem
   * @param side 1 = right side, 2 = left side
   */
  private final int side;
  public MotorTestCommand(DriveTrainSubsystem dsubsystem, int side) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrainSubsystem = dsubsystem;
    this.side = side;
    addRequirements(dsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(side){
      case 2:
        drivetrainSubsystem.moveLeftTrain(Constants.MOTOR_TESTING_SPEED);
        break;
      case 1:
        drivetrainSubsystem.moveRightTrain(Constants.MOTOR_TESTING_SPEED);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
