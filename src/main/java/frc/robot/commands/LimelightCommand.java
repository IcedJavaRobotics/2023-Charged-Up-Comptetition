// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;


public class LimelightCommand extends CommandBase {

private final LimelightSubsystem limelightSubsystem;
  private boolean targetFound = false;
  private double tx = 0.0;
  private double ty = 0.0;
  private double tid = 0.0;
  private double ta = 0.0;
  private double tv = 0.0;
  private double distance = 0.0;

  private double[] botpose = {0,0,0,0,0,0};
  private double range = 10;

  /*  Creates a new Limelight. */
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

//Whether the limelight has any valid targets (0 or 1)
tv = limelightSubsystem.getTv();
//Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
tx = limelightSubsystem.getTx();
//Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
ty = limelightSubsystem.getTy();
//Target Area (0% of image to 100% of image)
ta = limelightSubsystem.getTa();
//ID of primary AprilTag
tid = limelightSubsystem.getTid();
//distance
distance = limelightSubsystem.getDistance();

//botpose: Robot transform in field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
botpose = limelightSubsystem.getBotpose();


//translating tv into targetFound for convenience
if (tv == 0){targetFound = false;}else{targetFound = true;}





//Runs only if AprilTag is detected
if(targetFound){

      if(tx>=range){
        //runs if apriltag is on left to center
        limelightSubsystem.rotateRight();

      }else if(tx<=-range){
        //runs if apriltag is on right to center
        limelightSubsystem.rotateLeft();

      }else{
        //runs if apriltag is in the center of the screen(y is ignored)

        if(ta<=1.5){
          //if tag is small/far
          limelightSubsystem.driveForward();
        }else{
          //if tag is close up
          if(tx>=-1 && tx<=1){
            //if the tag is perfectly centered with the crosshairs x
            limelightSubsystem.stopAndDestroy();
          }else{
            //if you are close, but not quite centered perfectly.
            limelightSubsystem.stopAndSeek();
          }

        }
        
      }


}else{
  //turns red if it doesnt see apriltag
  limelightSubsystem.searchingForTargets();

}


//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", tx);
SmartDashboard.putNumber("LimelightY", ty);
SmartDashboard.putNumber("LimelightArea", ta);
SmartDashboard.putNumber("LimelightTV", tv);
SmartDashboard.putNumber("AprilTagID", tid);
SmartDashboard.putBoolean("TargetSpotted", targetFound);
SmartDashboard.putNumber("Distance",distance);

SmartDashboard.putNumberArray("Botpose", botpose);




}//end of execute

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

  

  

