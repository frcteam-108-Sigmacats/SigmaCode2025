// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.Key;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevatorconstants;

public class Elevator extends SubsystemBase {
  //Instantiating the elevator motors
 public SparkMax leftElevatorMotor;
 public SparkMax rightElevatorMotor;
 
 //Instantiating the Elevator Motor Configurations
 private SparkMaxConfig leftElevatorMotorConfig = new SparkMaxConfig();
 private SparkMaxConfig rightElevatorMotorConfig = new SparkMaxConfig();

 //Setting the reef score level to L1
 private int reefScoreLevel = 1;

 //Instantiating the PID Controller to postion our elevator smoothly and quickly
 private SparkClosedLoopController elevatorPIDController;
 
 //Instantiating a Relative Encoder to track our elevator position
 private RelativeEncoder elevatorencoder;
  /** Creates a new ExampleSubsystem. */
  public Elevator() {

    //Assinging our Elevator Motors their CAN IDs and Motor Types
   leftElevatorMotor = new SparkMax(Elevatorconstants.leftElevatorMotorID, MotorType.kBrushless);
   rightElevatorMotor = new SparkMax(Elevatorconstants.rightElevatorMotorID, MotorType.kBrushless);

   //Setting the Right Elevator Idle Mode and Current Limit
   rightElevatorMotorConfig.idleMode(IdleMode.kBrake);
   rightElevatorMotorConfig.smartCurrentLimit(Elevatorconstants.ElevatorMotorCurrentlimit);

   //Setting the CPR of our Alternate Encoder for accurate readings
   rightElevatorMotorConfig.alternateEncoder.countsPerRevolution(8192);
   rightElevatorMotorConfig.alternateEncoder.inverted(true);//Inverting our encoder to read the position of the elevator the right way

   //Assinging the PID Values to the Right Elevator PID Controller
   rightElevatorMotorConfig.closedLoop.pid(Elevatorconstants.piviotP, Elevatorconstants.piviotI, Elevatorconstants.piviotD);
   rightElevatorMotorConfig.closedLoop.velocityFF(1/5676);//Setting the Feedforward Voltage 
   rightElevatorMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);//Telling our PID Controller to use the Through Bore Encoder connected to the motor as feedback
   
   //Setting the Elevator Forward and Reverse Limits to make sure our Robot does not go beyond its limitations angle wise to not break the mechanism
   rightElevatorMotorConfig.softLimit.forwardSoftLimit(Elevatorconstants.elevatorForwardSoftLimit);
   rightElevatorMotorConfig.softLimit.reverseSoftLimit(Elevatorconstants.elevatorReverseSoftLimit);

   //Setting the Left Elevator Idle Mode and Current Limit
   leftElevatorMotorConfig.idleMode(IdleMode.kBrake);
   leftElevatorMotorConfig.smartCurrentLimit(Elevatorconstants.ElevatorMotorCurrentlimit);

   //Setting the Left Elevator Motor Forward and Reverse Limits
   leftElevatorMotorConfig.softLimit.forwardSoftLimit(Elevatorconstants.elevatorForwardSoftLimit);
   leftElevatorMotorConfig.softLimit.reverseSoftLimit(Elevatorconstants.elevatorReverseSoftLimit);

   //Telling the Left Elevator Motor to follow the Right Elevator Motor
   leftElevatorMotorConfig.follow(rightElevatorMotor, true);

   //Setting the PID Controller variable to the right elevator motor internal PID Controller
   elevatorPIDController = rightElevatorMotor.getClosedLoopController();

   //Setting the Relative Encoder variable to the Right Elevator Motor External connection to the Through Bore Encoder
   elevatorencoder = rightElevatorMotor.getAlternateEncoder();

   //Adding the configurations to the motor
   leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   //Resetting the elevator encoder to zero on boot up
   elevatorencoder.setPosition(0);
   
  }

//Setting the Elevator Position with PID Control
public void setElevatorPosition(double position){
  elevatorPIDController.setReference(position, ControlType.kPosition);
}

//Setting the Elevator Motors to a speed percentage
public void setElevatorSpeed(double Speed){
  rightElevatorMotor.set(Speed);
}

//Getting our Elevator Positiong
public double getElevatorPosition(){
  return elevatorencoder.getPosition();
}

//Setting the Reef Level variable (NOT USING ANYMORE)
public void setReefLevel(int level){
  reefScoreLevel = level;
}

//Getting which Level of the Reef we are on (NOT USING ANYMORE)
public int getReefLevel(){
  return reefScoreLevel;
}
  

/**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
  
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("Reef Level", reefScoreLevel);
  SmartDashboard.putNumber("Elevator position: ", getElevatorPosition());
  }
    // This method will be called once per scheduler run

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
