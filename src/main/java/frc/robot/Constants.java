// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Elevator;

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
  public static class Elevatorconstants{

    public static final int leftElevatorMotorID = 11 ;
    public static final int rightElevatorMotorID = 12 ;
      
    public static final int ElevatorMotorCurrentlimit = 40;

    public static final double piviotP = 0.001;
    public static final double piviotI = 0.0;
    public static final double piviotD = 0.0;
  
    public static final double elevatorForwardSoftLimit = 0;/*change when robo done */
    public static final double elevatorReverseSoftLimit = 0;
  }
}
