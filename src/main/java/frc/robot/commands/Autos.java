// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  public static CommandBase createAutonomousCommand(AutonomousStrategy autonomousStrategy, ExampleSubsystem subsystem, DriveTrainSubsystem msubsystem,
      ClawSubsystem csubsystem, ArmSubsystem asubsystem, ExtendoSubsystem esubsystem,
      PneumaticWheelsSubsystem pSubsystem, BlinkinSubsystem bSubsystem) {

        switch (autonomousStrategy) {
          case TAXI_ONLY:
            return Commands.sequence(new TaxiCommand(msubsystem));
          case TAXI_AND_BALANCE:
            return Commands.sequence(new TaxiCommand(msubsystem), new BalanceCommand(msubsystem));
          case SCORE_TAXI_AND_BALANCE:
            return Commands.sequence(new ScoreCommand(msubsystem), new TaxiCommand(msubsystem), new BalanceCommand(msubsystem));
        }

        return null;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
