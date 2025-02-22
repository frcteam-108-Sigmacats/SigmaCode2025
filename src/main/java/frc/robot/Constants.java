// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
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
  public static class SwerveDriveConstants{
    public static final int fLDriveMotorID = 1;
    public static final int fLTurnMotorID = 2;
    public static final double fLAbsEncoderOffset = -Math.PI / 2;

    public static final int fRDriveMotorID = 3;
    public static final int fRTurnMotorID = 4;
    public static final double fRAbsEncoderOffset = 0;

    public static final int bLDriveMotorID = 5;
    public static final int bLTurnMotorID = 6;
    public static final double bLAbsEncoderOffset = Math.PI;

    public static final int bRDriveMotorID = 7;
    public static final int bRTurnMotorID = 8;
    public static final double bRAbsEncoderOffset = Math.PI / 2;

    public static final int driveMotorCurrentLimit = 50; /*in amps */
    public static final int turnmotorCurrentLimit = 20; /*in amps */

    public static final double krakenRPM = 6000.0;

    public static final int DriveMotorPinionTeeth = 14;

    public static final double wheelDiameter = Units.inchesToMeters(3);

    public static final double kDrivingMotorReduction = (45.0 * 21) / (DriveMotorPinionTeeth * 15);
// p0.9 i0.0 d0.0//
    public static final double drivemotorP = 1.5;
    public static final double drivemotorI = 0.0;
    public static final double drivemotorD = 0.0;
// p1 i0 d0//
    public static final double turnmotorP = 1.0;
    public static final double turnmotorI = 0.0;
    public static final double turnmotorD = 0.0;

    public static final double turningFactor = 2 * Math.PI;

    public static double trackWidth = Units.inchesToMeters(22.4375);
    public static double wheelbase = Units.inchesToMeters(22.4375);
    public static double kMaxSpeedMPS = 25;
    public final static double maxAngularspeed = 4 * Math.PI;
    public static boolean gyroReversed = false;
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelbase / 2.0, trackWidth/ 2.0),
      new Translation2d(wheelbase / 2.0, -trackWidth/ 2.0),
      new Translation2d(-wheelbase / 2.0, trackWidth/ 2.0),
      new Translation2d(-wheelbase / 2.0, -trackWidth/ 2.0));
    
      public static final double deadband = 0.2;
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
  public static class Elevatorconstants{

    public static final int leftElevatorMotorID = 11 ;
    public static final int rightElevatorMotorID = 12 ;
      
    public static final int ElevatorMotorCurrentlimit = 40;

    public static final double piviotP = 1.5;
    public static final double piviotI = 0.0;
    public static final double piviotD = 0.01;
  
    public static final double elevatorForwardSoftLimit = 5.2;/*change when robo done */
    public static final double elevatorReverseSoftLimit = 0;
    public static final double L1 = 1.0;
    public static final double L2 = 2.0;
    public static final double L3 = 3.0;
    public static final double L4 = 4.0;
    
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

  public static final double WristOuttakeVelocity = -2000;

  public static final double cxaMotorFeedVelocity = -4000;
  public static final double coralHopperSpeed = 0.5;
}


}

