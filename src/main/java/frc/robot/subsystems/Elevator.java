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
 public SparkMax leftElevatorMotor;
 public SparkMax rightElevatorMotor;
 
 private SparkMaxConfig leftElevatorMotorConfig = new SparkMaxConfig();
 private SparkMaxConfig rightElevatorMotorConfig = new SparkMaxConfig();

 private SparkClosedLoopController elevatorPIDController;
 
 private RelativeEncoder elevatorencoder;
  /** Creates a new ExampleSubsystem. */
  public Elevator() {

   leftElevatorMotor = new SparkMax(Elevatorconstants.leftElevatorMotorID, MotorType.kBrushless);
   rightElevatorMotor = new SparkMax(Elevatorconstants.rightElevatorMotorID, MotorType.kBrushless);

   rightElevatorMotorConfig.idleMode(IdleMode.kBrake);
   rightElevatorMotorConfig.smartCurrentLimit(Elevatorconstants.ElevatorMotorCurrentlimit);

   rightElevatorMotorConfig.alternateEncoder.countsPerRevolution(8192);
   rightElevatorMotorConfig.alternateEncoder.inverted(true);
   rightElevatorMotorConfig.closedLoop.pid(Elevatorconstants.piviotP, Elevatorconstants.piviotI, Elevatorconstants.piviotD);
   rightElevatorMotorConfig.closedLoop.velocityFF(1/5676);
   rightElevatorMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
   
   rightElevatorMotorConfig.softLimit.forwardSoftLimit(Elevatorconstants.elevatorForwardSoftLimit);
   rightElevatorMotorConfig.softLimit.reverseSoftLimit(Elevatorconstants.elevatorReverseSoftLimit);

   leftElevatorMotorConfig.idleMode(IdleMode.kBrake);
   leftElevatorMotorConfig.smartCurrentLimit(Elevatorconstants.ElevatorMotorCurrentlimit);

   leftElevatorMotorConfig.softLimit.forwardSoftLimit(Elevatorconstants.elevatorForwardSoftLimit);
   leftElevatorMotorConfig.softLimit.reverseSoftLimit(Elevatorconstants.elevatorReverseSoftLimit);

   leftElevatorMotorConfig.follow(rightElevatorMotor, true);

   elevatorPIDController = rightElevatorMotor.getClosedLoopController();


   elevatorencoder = rightElevatorMotor.getAlternateEncoder();

   leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   elevatorencoder.setPosition(0);
   
  }
public void setElevatorPosition(double position){
  elevatorPIDController.setReference(position, ControlType.kPosition);

 
}
public void setElevatorSpeed(double Speed){
rightElevatorMotor.set(Speed);}

public double getElevatorPosition(){
  return elevatorencoder.getPosition();
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
  SmartDashboard.putNumber("Elevator position: ", getElevatorPosition());
  }
    // This method will be called once per scheduler run

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
