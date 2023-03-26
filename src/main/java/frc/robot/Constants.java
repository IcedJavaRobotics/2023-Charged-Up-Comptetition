// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Drive train
  public static final int FRONT_LEFT_TALON = 4;
  public static final int FRONT_RIGHT_TALON = 1;
  public static final int BACK_LEFT_TALON = 3;
  public static final int BACK_RIGHT_TALON = 2;
  public static final int DROP_WHEEL_SPARK = 9;
  public static final int ROTATIONAL_CONSTANT = 2048;

  // Auto Drive train
  public static final int AUTO_TAXI_DISTANCE = 170; // in inches
  public static final double AUTO_TAXI_SPEED = 0.5;
  public static final double AUTO_SCORING_SPEED = -0.5;
  public static final int AUTO_SCORING_DISTANCE = 6;
  public static final double CHARGING_MAX_ANGLE = 101;
  public static final double CHARGING_MIN_ANGLE = 75;   // Theoretical

  public static final double AUTO_CHARGING_SPEED = 0.15;

  // Controllers
  public static final int JOYSTICK = 0;
  public static final int DRIVER_STATION = 1;
  public static final int CONTROLLER = 2;
  // ^ these indicate the spot used on the driverstation ^
  public static final double DEADZONE = 0.2;

  // Claw
  public static final int LEFT_CLAW = 7;
  public static final int RIGHT_CLAW = 8;
  public static final double CLAW_SPEED = 1;

  // Digital inputs
  public static final int RIGHT_CLAW_LIMIT = 0;
  public static final int LEFT_CLAW_LIMIT = 1;
  public static final int EXTENDO_LIMIT_SWITCH = 2;
  public static final int ARM_LIMIT_SWITCH = 3;

  // Buttons
  public static final int CLAW_CLOSE_BUTTON = 7;
  public static final int CLAW_OPEN_BUTTON = 8;

  // Extendo
  public static final int EXTENDO_MOTOR = 5;
  public static final double EXTENDO_SPEED = 0.3; // changed from 0.6 to 0.3 // Encoder values for set arm positions
  public static final double EXTENDO_RETRACT_SPEED = 0.7;
  public static final int DEFAULT_SETPOINT = 0;

  public static final int EXTENDO_UPPER_CUBE_SETPOINT = 17000;
  public static final int EXTENDO_MIDDLE_CUBE_SETPOINT = 2900;
  public static final int EXTENDO_UPPER_CONE_SETPOINT = 16000;
  public static final int EXTENDO_MIDDLE_CONE_SETPOINT = 1800;

  // Arm
  public static final int ARM_SPARK = 6;
  public static final double ARM_SPEED = 1;
  public static final int ARM_UPPER_CUBE_SETPOINT = 181;
  public static final int ARM_MIDDLE_CUBE_SETPOINT = 130;
  public static final int ARM_UPPER_CONE_SETPOINT = 205;
  public static final int ARM_MIDDLE_CONE_SETPOINT = 172;

  // Limelight
  /** upward angle of limelight camera [degrees] */
  public static final double LIMELIGHT_ANGLE = 3.0;
  /** distance from limelight lens from floor [inches] */
  public static final double LIMELIGHT_HEIGHT = 18.5;
  /** distance from apriltag to floor(bottom of tag) [inches] */
  public static final double APRILTAG_HEIGHT = 14.25;
  /**
   * distance from apriltag to floor but its the double substation(bottom of tag)
   * [inches]
   */
  public static final double APRILTAG_DOUBLE_SUBSTATION_HEIGHT = 23.375;

}
