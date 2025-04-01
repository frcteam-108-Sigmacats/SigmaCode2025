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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntake extends SubsystemBase {
  //Instantiating the motors for the Ground Algae Intake mechanisms
  private SparkMax algaePivotMotor;//The motor that pivots the algae intake in and out of the robot
  private SparkMax algaeRollerMotor;//The motor that runs the roller on the algae intake to intake algae from the ground
  private SparkMax algaeClimbMotor;//The motor that releases the hopper and tensions the rope tied to the algae intake for climbing

  //Instantiating the configurations for the motors on the ground algae intake
  private SparkMaxConfig algaePivotMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig alageRollerMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig algaeClimbMotorConfig = new SparkMaxConfig();

  //Instantiating the PID Controller to use position control for the algae pivot motor for smooth and quick positioning of the Ground Algae Intake Mechanism
  private SparkClosedLoopController pivotPIDController;

  //Instantiating the absolute encoder that tracks the angle of the Ground Algae Mechanism 
  private AbsoluteEncoder pivotAbsEncoder;

  //A boolean to check if we have an algae to constantly hold the algae in the rest position
  private boolean isThereAlgae;
  /** Creates a new ExampleSubsystem. */
  public AlgaeIntake() {
    //Assigning the motors their CAN IDs and Motor Type
    algaePivotMotor = new SparkMax(AlgaeIntakeConstants.algaePivotMotorID, MotorType.kBrushless);
    algaeRollerMotor = new SparkMax(AlgaeIntakeConstants.algaeRollerMotorID, MotorType.kBrushless);
    algaeClimbMotor = new SparkMax(AlgaeIntakeConstants.algaeClimbMotorID, MotorType.kBrushless);
    
    //Setting the Algae Pivot Idle Mode and Current Limit for the motor
    algaePivotMotorConfig.idleMode(IdleMode.kBrake);
    algaePivotMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.algaePivotMotorCurrentLimit);

    //Setting the Algae Climb Idle Mode and Current Limit for the motor
    algaeClimbMotorConfig.idleMode(IdleMode.kBrake);
    algaeClimbMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.algaeClimbMotorCurrentLimit);

    //Setting the absolute encoder to track the angle of the pivot in degrees and inverting the absolute encoder
    algaePivotMotorConfig.absoluteEncoder.positionConversionFactor(360);
    algaePivotMotorConfig.absoluteEncoder.inverted(true);

    //Setting the Algae PID Configs with FeedForward Voltage
    algaePivotMotorConfig.closedLoop.pid(AlgaeIntakeConstants.pivotP, AlgaeIntakeConstants.pivotI, AlgaeIntakeConstants.pivotD);
    algaePivotMotorConfig.closedLoop.velocityFF(1/5676);
    algaePivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);//Saying to track the absolute encoder for the angle
    //Telling the PID Controller to get to the angle the fastest way possible. So if we need to be at 270 and we are at 40 we turn counter clockwise since its faster
    algaePivotMotorConfig.closedLoop.positionWrappingEnabled(true);
    algaePivotMotorConfig.closedLoop.positionWrappingMinInput(0);
    algaePivotMotorConfig.closedLoop.positionWrappingMaxInput(360);

    //Giving a safeguard where the motor will not try to run counterclockwise if the angle of the pivot is 0
    algaePivotMotorConfig.softLimit.reverseSoftLimit(0);

    //Assigning the PIDController and Absolute Encoder variable to the motor holding the components
    pivotPIDController = algaePivotMotor.getClosedLoopController();
    pivotAbsEncoder = algaePivotMotor.getAbsoluteEncoder();

    //Setting the Algae Roller IdleMode, Current Limit, and inverting it since it is placed backwards
    alageRollerMotorConfig.idleMode(IdleMode.kCoast);
    alageRollerMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.algaeRollerMotorCurrentLimit);
    alageRollerMotorConfig.inverted(true);

    //Setting the motor configurations
    algaePivotMotor.configure(algaePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeRollerMotor.configure(alageRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeClimbMotor.configure(algaeClimbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Setting the boolean to false at the beginning since we will never be holding an algae at the start of the match
    isThereAlgae = false;
  }
  //Setting the Algae Pivot with a speed percentage
  public void SetAlgaeIntakePivotSpeed(double speed){
    algaePivotMotor.set(speed);
  }

  //Setting the position for the Algae using PID Control
  public void setAlgaePivot(double position){
    pivotPIDController.setReference(position, ControlType.kPosition);
  }

  //Setting a speed for the climb motor
  public void setClimbMotorSpeed(double speed){
    algaeClimbMotor.set(speed);
  }

  //Getting the Algae Mechanism angle
  public double getAlgaePivotPosition(){
    return pivotAbsEncoder.getPosition();
  }

  //Setting a speed percentage for the Algae Roller
  public void setAlgaeRollerSpeed(double speed){
    algaeRollerMotor.set(speed);
  }

  //Setting the boolean of if we do have an algae or not
  public void setAlgaeBool(boolean ag){
    isThereAlgae = ag;
  }

  //Checking if we do have an algae with our boolean
  public boolean doWeHaveAlgae(){
    return isThereAlgae;
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
    SmartDashboard.putNumber("algae Pivot Position", getAlgaePivotPosition());
    SmartDashboard.putNumber("Get Current: ", algaeRollerMotor.getOutputCurrent());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public double getAlgaeRollerCurrent(){
    return algaeRollerMotor.getOutputCurrent();
  }
}
