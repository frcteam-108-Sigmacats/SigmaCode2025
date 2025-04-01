// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
  // private NetworkTable

  /** Creates a new vision. */
  public Vision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Setting our Robot Orientation for getting accurate Pose Estimation
    LimelightHelpers.SetRobotOrientation("limelight-leftll",
    SwerveDrive.getPoseForVision().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    LimelightHelpers.SetRobotOrientation("limelight-rightll",
    SwerveDrive.getPoseForVision().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    SmartDashboard.putBoolean("Left LL Tag Detected", istheretagLeftLL());
    SmartDashboard.putBoolean("Right LL Tag Detected", istheretagRightLL());
    SmartDashboard.putNumber("Left LL Tag ID", (int)getLeftLLTagID());
    SmartDashboard.putNumber("Right LL Tag Detected", (int)getRightLLTagID());

  }

  //Getting our X Offset from the Left Limelight Crosshair
  public double getLeftLLTX() {
    return LimelightHelpers.getTX("limelight-leftll");
  }

  //Getting our X Offset from the Right Limelight Crosshair
  public double getRightLLTX() {
    return LimelightHelpers.getTX("limelight-rightll");
  }

  //Getting our Y Offset from our Left Limelight Crosshair
  public double getLeftLLTY() {
    return LimelightHelpers.getTY("limelight-leftll");

  }

  //Getting our Y Offset from our Right Limelight Crosshair
  public double getRightLLTY() {
    return LimelightHelpers.getTY("limelight-rightll");

  }

  //Getting the Area that the Left Limelight sees
  public double getLeftLLTA() {
    return LimelightHelpers.getTA("limelight-leftll");
  }

  //Getting the Area that the Right Limelight sees
  public double getRightLLTA() {
    return LimelightHelpers.getTA("limelight-rightll");
  }

  //Checking to see if our Left Limelight sees an AprilTag
  public boolean istheretagLeftLL() {
    return LimelightHelpers.getTV("limelight-leftll");

  }

  //Checking to see if our Right Limelight sees and Apriltag
  public boolean istheretagRightLL() {
    return LimelightHelpers.getTV("limelight-rightll");
  }

  //Checking which AprilTag our Left Limelight sees
  public double getLeftLLTagID(){
    return LimelightHelpers.getFiducialID("limelight-leftll");
  }

  //Checking which AprilTag our Right Limelight sees
  public double getRightLLTagID(){
    return LimelightHelpers.getFiducialID("limelight-rightll");
  }

  //Getting our Left Limelights Robot Pose Estimation
  public PoseEstimate getLeftLLBotPose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-leftll");
  }

  //Getting our Right Limelights Robot Pose Estimation
  public PoseEstimate getRightLLBotPose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rightll");
  }

  //Getting our Left Limelight's Target Pose in Robot Space
  public double[] getLeftLLTargetBotPose() {
    return LimelightHelpers.getTargetPose_RobotSpace("limelight-leftll");
  }

  //Getting our Right Limelight's Target Pose in Robot Space
  public double[] getRightLLTargetBotPose() {
    return LimelightHelpers.getTargetPose_RobotSpace("limelight-rightll");
  }
}
