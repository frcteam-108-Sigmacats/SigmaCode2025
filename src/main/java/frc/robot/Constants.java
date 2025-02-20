// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static class AlgaeIntakeConstants{
    public static final int algaePivotMotorID = 9;
    public static final int algaeRollerMotorID = 10;

    public static final double pivotP = 0.01;
    public static final double pivotI = 0.0;
    public static final double pivotD = 0.0;

    public static final int algaePivotMotorCurrentLimit = 20; //Units in amps
    public static final int algaeRollerMotorCurrentLimit = 15; //Units in amps

    public static final double algaeIntakePivotPosition = 42;
    public static final double algaeRestPivotPosition = 0.0;
    public static final double algaeOuttakePivotPosition = 42;

    public static final double algaeIntakeSpeed = 0.45;
    public static final double algaeRestSpeed = 0.0;
    public static final double algaeOuttakeSpeed = -0.3;
  }
}
