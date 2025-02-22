// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
  private SwerveDrive swervesub = new SwerveDrive();

  /** Creates a new vision. */
  public Vision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LimelightHelpers.SetRobotOrientation("LeftLL",
        swervesub.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);

    LimelightHelpers.SetRobotOrientation("RightLL",
        swervesub.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
  }

  public double getLeftLLTX() {
    return LimelightHelpers.getTX("LeftLL");
  }

  public double getRightLLTX() {
    return LimelightHelpers.getTX("RightLL");
  }

  public double getLeftLLTY() {
    return LimelightHelpers.getTY("LeftLL");

  }

  public double getRightLLTY() {
    return LimelightHelpers.getTY("RightLL");

  }

  public double getLeftLLTA() {
    return LimelightHelpers.getTA("LeftLL");
  }

  public double getRightLLTA() {
    return LimelightHelpers.getTA("RightLL");
  }

  public boolean istheretagLeftLL() {
    return LimelightHelpers.getTV("LeftLL");

  }

  public boolean istheretagRightLL() {
    return LimelightHelpers.getTV("RightLL");
  }

  public PoseEstimate getLeftLLBotPose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("LeftLL");
  }
  public PoseEstimate getRightLLBotPose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("RightLL");
  }
}
