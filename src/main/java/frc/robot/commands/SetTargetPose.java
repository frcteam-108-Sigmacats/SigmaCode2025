// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetTargetPose extends InstantCommand {
  private SwerveDrive swerveSub;
  private Vision visionSub;
  private boolean left;
  public SetTargetPose(SwerveDrive swerveSub, Vision visionSub, boolean left) {
    this.swerveSub = swerveSub;
    this.visionSub = visionSub;
    this.left = left;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d targetPose;
    if(swerveSub.getAllianceColor()){
      if(left){
        if(visionSub.istheretagRightLL()){
          targetPose = VisionConstants.leftBluePoses.get((int) visionSub.getLeftLLTagID());
        }
        else if(visionSub.istheretagLeftLL()){
          targetPose = VisionConstants.leftBluePoses.get((int) visionSub.getRightLLTagID());
        }
        else{
          targetPose = new Pose2d();
        }
        
      }
      else{
        if(visionSub.istheretagLeftLL()){
          targetPose = VisionConstants.rightBluePoses.get((int) visionSub.getLeftLLTagID());
        }
        else if(visionSub.istheretagRightLL()){
          targetPose = VisionConstants.rightBluePoses.get((int) visionSub.getRightLLTagID());
        }
        else{
          targetPose = new Pose2d();
        }
        
      }
    }
    else{
      if(left){
        if(visionSub.istheretagLeftLL()){
          targetPose = VisionConstants.leftRedPoses.get((int) visionSub.getLeftLLTagID());
        }
        else if(visionSub.istheretagRightLL()){
          targetPose = VisionConstants.leftRedPoses.get((int) visionSub.getRightLLTagID());
        }
        else{
          targetPose = new Pose2d();
        }
        
      }
      else{
        if(visionSub.istheretagLeftLL()){
          targetPose = VisionConstants.rightRedPoses.get((int) visionSub.getLeftLLTagID());
        }
        else if(visionSub.istheretagRightLL()){
          targetPose = VisionConstants.rightRedPoses.get((int) visionSub.getRightLLTagID());
        }
        else{
          targetPose = new Pose2d();
        }
        
      }
    }

    SmartDashboard.putNumberArray("DriveToPose", new double[] {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getRadians()});
    swerveSub.setTargetPose(targetPose);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
