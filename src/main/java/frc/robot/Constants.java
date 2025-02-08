// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
public static final class CoralXAlgaeWristConstants{
  public static final int cXAMotorID = 13;
  public static final int coralHopperMotorID = 14;
  public static final int coralXAlgaeWristID = 15;

  public static final int coralDetectorID = 1;

  public static final double pivotP = 0.001;
  public static final double pivotI = 0.0;
  public static final double pivotD = 0.0;
  public static final double pivotFF = 1/5676;

  public static final double velocityP = 0.001;
  public static final double velocityI = 0.0;
  public static final double velocityD = 0.0;

  public static final int coralMotorCurrentLimit = 40;
  public static final int coralHopperMotorCurrentLimit = 20;
  public static final int coralAlgaeWristCurrentLimit = 20;

  public static final double forwardSoftLimitWrist = 0.0;
  public static final double reverseSoftLimitWrist = 0.0;
}
}
