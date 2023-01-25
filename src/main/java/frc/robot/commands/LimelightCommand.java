// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;


public class LimelightCommand extends CommandBase {

private final LimelightSubsystem limelightSubsystem;
  private boolean targetFound = false;
  private double range = 10;
  private double tx = 0.0;
  private double ty = 0.0;
  private double tid = 0.0;
  private double ta = 0.0;
  private double tv = 0.0;
  private double camtran = 0.0;
  


  /* Creates a new Limelight. */
   public LimelightCommand(LimelightSubsystem subsystem) {
    limelightSubsystem=subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelightSubsystem);
  }

 
  @Override
  public void initialize() {
  //called when robot is first started up

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//variables for examples
  // final double STEER_K = 0.03;                    // how hard to turn toward the target
  // final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
  // final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
  // final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast


//Whether the limelight has any valid targets (0 or 1)
tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

//Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
//Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

//Target Area (0% of image to 100% of image)
ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

//ID of primary AprilTag
tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

// Camera transform in target space of primary apriltag or solvepnp target.
//  NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDouble(0);


//translating tv into targetFound for convenience
if (tv == 0){targetFound = false;}else{targetFound = true;}

//Runs only if AprilTag is detected
if(targetFound){

      if(tx>=range){
        //runs if apriltag is on left to center
        rotateRight();

      }else if(tx<=-range){
        //runs if apriltag is on right to center
        rotateLeft();

      }else{
        //runs if apriltag is in the center of the screen(y is ignored)

        if(ta<=1.5){
          //if tag is small/far
        driveForward();
        }else{
          //if tag is close up
          if(tx>=-1 && tx<=1){
            //if the tag is perfectly centered with the crosshairs x
          stopAndDestroy();
          }else{
            //if you are close, but not quite centered perfectly.
          stopAndSeek();
          }

        }
        
      }


}else{
  //turns red if it doesnt see apriltag
  searchingForTargets();

}

//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", tx);
SmartDashboard.putNumber("LimelightY", ty);
SmartDashboard.putNumber("LimelightArea", ta);
SmartDashboard.putNumber("LimelightTV", tv);
SmartDashboard.putNumber("AprilTagID", tid);
SmartDashboard.putBoolean("TargetSpotted", targetFound);


}//end of execute

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //Methods of what to do, robot is very polite
  public void rotateRight(){
    limelightSubsystem.turnDarkBlue();
    System.out.println("A little right please");
  }
  public void rotateLeft(){
    limelightSubsystem.turnLightLightBlue();
    System.out.println("A little left please");
  }
  public void driveForward(){
    //when you have apriltag centered but far
    limelightSubsystem.turnGreen();
    System.out.println("GO GO GO");
  }
  public void stopAndSeek(){
    //when you are close but not perfectly centered
    limelightSubsystem.flashRed();
    if(tx>=1){
    System.out.println("Seeking TARGET...Turn LEFT please.");
    }if(tx<=-1){
      System.out.println("Seeking TARGET...Turn RIGHT please.");
    }
  }
  public void stopAndDestroy(){
    //when you are close and perfectly centered
    limelightSubsystem.turnDarkGreen();
    System.out.println("DESTROYING APRILTAG "+tid);
  }
  public void searchingForTargets(){
    //no apriltags seen
    limelightSubsystem.turnRed();
    System.out.println("Scanning for Targets....");
  }
}