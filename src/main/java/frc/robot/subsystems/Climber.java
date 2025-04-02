// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  //Instantiating the motors for the Ground Algae Intake mechanisms
  private SparkMax climberWinchMotor;//The motor that pivots the algae intake in and out of the robot
  private SparkMax climberIntakeMotor;//The motor that runs the roller on the algae intake to intake algae from the ground
  //Instantiating the configurations for the motors on the ground algae intake
  private SparkMaxConfig climberWinchMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig climberIntakeMotorConfig = new SparkMaxConfig();

  //Instantiating the PID Controller to use position control for the algae pivot motor for smooth and quick positioning of the Ground Algae Intake Mechanism
  private SparkClosedLoopController pivotPIDController;

  //Instantiating the absolute encoder that tracks the angle of the Ground Algae Mechanism 
  private AbsoluteEncoder pivotAbsEncoder;

  private Servo ratchetServo = new Servo(0);

  private DigitalInput isCageIn = new DigitalInput(9);
  /** Creates a new ExampleSubsystem. */
  public Climber() {
    //Assigning the motors their CAN IDs and Motor Type
    climberWinchMotor = new SparkMax(ClimberConstants.climberWinchMotorID, MotorType.kBrushless);
    climberIntakeMotor = new SparkMax(ClimberConstants.climberIntakeMotorID, MotorType.kBrushless);
    
    //Setting the Algae Pivot Idle Mode and Current Limit for the motor
    climberWinchMotorConfig.idleMode(IdleMode.kBrake);
    climberWinchMotorConfig.smartCurrentLimit(ClimberConstants.climberWinchMotorCurrentLimit);

    //Setting the absolute encoder to track the angle of the pivot in degrees and inverting the absolute encoder
    climberWinchMotorConfig.absoluteEncoder.positionConversionFactor(360);
    climberWinchMotorConfig.absoluteEncoder.inverted(true);

    //Setting the Algae PID Configs with FeedForward Voltage
    climberWinchMotorConfig.closedLoop.pid(ClimberConstants.pivotP, ClimberConstants.pivotI, ClimberConstants.pivotD);
    climberWinchMotorConfig.closedLoop.velocityFF(1/5676);
    climberWinchMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);//Saying to track the absolute encoder for the angle
    //Telling the PID Controller to get to the angle the fastest way possible. So if we need to be at 270 and we are at 40 we turn counter clockwise since its faster
    climberWinchMotorConfig.closedLoop.positionWrappingEnabled(true);
    climberWinchMotorConfig.closedLoop.positionWrappingMinInput(0);
    climberWinchMotorConfig.closedLoop.positionWrappingMaxInput(360);
    //Giving a safeguard where the motor will not try to run counterclockwise if the angle of the pivot is 0
    climberWinchMotorConfig.softLimit.reverseSoftLimit(0);

    //Assigning the PIDController and Absolute Encoder variable to the motor holding the components
    pivotPIDController = climberWinchMotor.getClosedLoopController();
    pivotAbsEncoder = climberWinchMotor.getAbsoluteEncoder();

    //Setting the Algae Roller IdleMode, Current Limit, and inverting it since it is placed backwards
    climberIntakeMotorConfig.idleMode(IdleMode.kBrake);
    climberIntakeMotorConfig.smartCurrentLimit(ClimberConstants.climberIntakeMotorCurrentLimit);
    climberIntakeMotorConfig.inverted(true);

    //Setting the motor configurations
    climberWinchMotor.configure(climberWinchMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climberIntakeMotor.configure(climberIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  }
  //Setting the Algae Pivot with a speed percentage
  public void setClimberWinchSpeed(double speed){
    climberWinchMotor.set(speed);
  }

  //Setting the position for the Algae using PID Control
  public void setClimberPosition(double position){
    pivotPIDController.setReference(position, ControlType.kPosition);
  }
 

  public void stopClimberWinchMotor(){
    climberWinchMotor.setVoltage(0);
  }

  //Getting the Algae Mechanism angle
  public double getClimbPosition(){
    return pivotAbsEncoder.getPosition();
  }

  //Setting a speed percentage for the Algae Roller
  public void setClimberIntakeSpeed(double speed){
    climberIntakeMotor.set(speed);
  }

  public void setServoSpeed(double speed){
   ratchetServo.setSpeed(speed);
  }

  public void setServoPosition(double position){
    ratchetServo.set(position);
  }

  public boolean isCageIn(){
    return isCageIn.get();
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
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("algae Pivot Position", getClimbPosition());  
    System.out.println("Is Cage In " + isCageIn.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
