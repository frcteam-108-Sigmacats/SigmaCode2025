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
    LimelightHelpers.SetRobotOrientation("limelight-leftll",
    SwerveDrive.getPoseForVision().getRotation().getDegrees(), 0, 0, 0, 0, 0);

LimelightHelpers.SetRobotOrientation("limelight-rightll",
    SwerveDrive.getPoseForVision().getRotation().getDegrees(), 0, 0, 0, 0, 0);
  SmartDashboard.putBoolean("Left LL Tag Detected", istheretagLeftLL());
  SmartDashboard.putBoolean("Right LL Tag Detected", istheretagRightLL());
  SmartDashboard.putNumber("Left LL Tag ID", (int)getLeftLLTagID());
  SmartDashboard.putNumber("Right LL Tag Detected", (int)getRightLLTagID());

  }

  public double getLeftLLTX() {
    return LimelightHelpers.getTX("limelight-leftll");
  }

  public double getRightLLTX() {
    return LimelightHelpers.getTX("limelight-rightll");
  }

  public double getLeftLLTY() {
    return LimelightHelpers.getTY("limelight-leftll");

  }

  public double getRightLLTY() {
    return LimelightHelpers.getTY("limelight-rightll");

  }

  public double getLeftLLTA() {
    return LimelightHelpers.getTA("limelight-leftll");
  }

  public double getRightLLTA() {
    return LimelightHelpers.getTA("limelight-rightll");
  }

  public boolean istheretagLeftLL() {
    return LimelightHelpers.getTV("limelight-leftll");

  }

  public boolean istheretagRightLL() {
    return LimelightHelpers.getTV("limelight-rightll");
  }

  public double getLeftLLTagID(){
    return LimelightHelpers.getFiducialID("limelight-leftll");
  }

  public double getRightLLTagID(){
    return LimelightHelpers.getFiducialID("limelight-rightll");
  }

  public PoseEstimate getLeftLLBotPose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-leftll");
  }
  public PoseEstimate getRightLLBotPose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rightll");
  }
  public double[] getLeftLLTargetBotPose() {
    return LimelightHelpers.getTargetPose_RobotSpace("limelight-leftll");
  }
  public double[] getRightLLTargetBotPose() {
    return LimelightHelpers.getTargetPose_RobotSpace("limelight-rightll");
  }
}
