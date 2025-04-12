// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralXAlgaeWristConstants;

public class CoralXAlgaeMech extends SubsystemBase {
  //Instantiating the Motor that will remove algae on the wrist and scoring coral on the reef (Both in one)
  private TalonFX cXAMotor;

  //Instantiating the Motor on the hopper to put in our wrist
  private SparkMax coralHopperMotor;

  //Instantiating the Motor that rotates the wrist in and out of the robot to score on the reef or net and remove algae
  private SparkFlex coralAlgaeWristMotor;

  //Instantiating configurations for CXA (Coral X Algae) Motor
  private TalonFXConfiguration cXAMotorConfig = new TalonFXConfiguration();
  private VelocityVoltage velocity = new VelocityVoltage(0);

  //Instantiating the configurations for the Hopper Motor and Wrist Motor
  private SparkMaxConfig coralHopperMotorConfig = new SparkMaxConfig();
  private SparkFlexConfig coralAlgaeWristMotorConfig = new SparkFlexConfig();

  //Instantiating the Absolute Encoder that will track the angle of the Wrist
  private AbsoluteEncoder pivotAbsEncoder;

  //Instantiating the PID Controller to control the position of the Wrist
  private SparkClosedLoopController wristPIDController;

  //Instantiating the CANRange that will help us detect our Coral in the Wrist and controlling the position 
  private CANrange coralDetector;

  //Instantiating the CANRange configurations 
  private CANrangeConfiguration coralDetectorConfig = new CANrangeConfiguration();

  //Instantiating the boolean to check if we have Algae in our Wrist
  private boolean doWeHaveAlgae;

  private boolean isThereCoral;
  /** Creates a new ExampleSubsystem. */
  public CoralXAlgaeMech() {
    //Assigning the CAN ID and CAN Bus of where the CXA Motor is and restarting the motor configurations to default
    cXAMotor = new TalonFX(CoralXAlgaeWristConstants.cXAMotorID, "*");
    cXAMotor.getConfigurator().apply(new TalonFXConfiguration());

    //Assigning the CAN ID and Motor Type to the Hopper Motor and Wrist Motor 
    coralHopperMotor = new SparkMax(CoralXAlgaeWristConstants.coralHopperMotorID, MotorType.kBrushless);
    coralAlgaeWristMotor = new SparkFlex(CoralXAlgaeWristConstants.coralXAlgaeWristID, MotorType.kBrushless);

    //Assinging the CAN ID and CAN Bus of the CAN Range
    coralDetector = new CANrange(CoralXAlgaeWristConstants.coralDetectorID, "*");
    coralDetector.getConfigurator().apply(new CANrangeConfiguration());

    //Setting the Range of where the CANRange will detect the coral with the center FOV being more up from the center
    coralDetectorConfig.FovParams.FOVRangeY = 11;
    coralDetectorConfig.FovParams.FOVRangeX = 11;
    coralDetectorConfig.FovParams.FOVCenterY = 10;
    coralDetectorConfig.ProximityParams.ProximityThreshold = 0.12;//Setting the proximity distance of the CANRange

    cXAMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;//Setting the Neutral Mode of the CXA Motor
    cXAMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;//Enabling Current Limit
    cXAMotorConfig.CurrentLimits.StatorCurrentLimit = CoralXAlgaeWristConstants.coralMotorCurrentLimit;//Setting Current Limit for the CXA Motor
    cXAMotorConfig.Slot0.kP = CoralXAlgaeWristConstants.velocityP;//Setting the P value for the CXA Motor Velocity Control
    cXAMotorConfig.Slot0.kI = CoralXAlgaeWristConstants.velocityI;//Setting the I value for the CXA Motor Velocity Control
    cXAMotorConfig.Slot0.kD = CoralXAlgaeWristConstants.velocityD;//Setting the D value for the CXA Motor Velocity Control
    cXAMotorConfig.Slot1.kP = CoralXAlgaeWristConstants.fastVelocityP;
    cXAMotorConfig.Slot1.kI = CoralXAlgaeWristConstants.fastVelocityI;
    cXAMotorConfig.Slot1.kD = CoralXAlgaeWristConstants.fastVelocityD;

    //Setting the Coral Hopper Motor Idle Mode and Current Limit
    coralHopperMotorConfig.idleMode(IdleMode.kCoast);
    coralHopperMotorConfig.smartCurrentLimit(CoralXAlgaeWristConstants.coralHopperMotorCurrentLimit);

    coralHopperMotorConfig.signals.absoluteEncoderPositionAlwaysOn(false);
    coralHopperMotorConfig.signals.absoluteEncoderVelocityAlwaysOn(false);
    coralHopperMotorConfig.signals.analogPositionAlwaysOn(false);
    coralHopperMotorConfig.signals.externalOrAltEncoderPositionAlwaysOn(false);
    coralHopperMotorConfig.signals.externalOrAltEncoderVelocityAlwaysOn(false);
    coralHopperMotorConfig.signals.iAccumulationAlwaysOn(false);
    coralHopperMotorConfig.signals.primaryEncoderPositionAlwaysOn(false);
    
    //Setting the Wrist Idle Mode and Current Limit
    coralAlgaeWristMotorConfig.idleMode(IdleMode.kBrake);
    coralAlgaeWristMotorConfig.smartCurrentLimit(CoralXAlgaeWristConstants.coralAlgaeWristCurrentLimit);
    
    //Setting the Absolute Encoder to track the Wrist angle in degrees and invert the Wrist
    coralAlgaeWristMotorConfig.absoluteEncoder.positionConversionFactor(360);
    coralAlgaeWristMotorConfig.absoluteEncoder.inverted(true);
    
    // coralAlgaeWristMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    // coralAlgaeWristMotorConfig.softLimit.reverseSoftLimitEnabled(true);

    // coralAlgaeWristMotorConfig.softLimit.forwardSoftLimit(CoralXAlgaeWristConstants.forwardSoftLimitWrist);
    // coralAlgaeWristMotorConfig.softLimit.reverseSoftLimit(CoralXAlgaeWristConstants.reverseSoftLimitWrist);

    //Setting the PID value for the Wrist Positiong Control
    coralAlgaeWristMotorConfig.closedLoop.pid(CoralXAlgaeWristConstants.pivotP, CoralXAlgaeWristConstants.pivotI, CoralXAlgaeWristConstants.pivotD);
    coralAlgaeWristMotorConfig.closedLoop.velocityFF(CoralXAlgaeWristConstants.pivotFF);//Setting a FeedForward Voltage
    coralAlgaeWristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);//Assigning the Position Control to track an Absolute Encoder connected to the motor
    coralAlgaeWristMotorConfig.closedLoop.positionWrappingEnabled(true);//Enabling PID Wrapping where we go to the angle the quickest way possible (basically bidirectional movement)
    coralAlgaeWristMotorConfig.closedLoop.positionWrappingInputRange(0, 360);//Setting the PID Wrapping to loop around 360
    

    //Assigning the PID Controller and Absolute Encoder variables to the motor holding the components
    wristPIDController = coralAlgaeWristMotor.getClosedLoopController();
    pivotAbsEncoder = coralAlgaeWristMotor.getAbsoluteEncoder();


    //Applying the configurations to the motors
    cXAMotor.getConfigurator().apply(cXAMotorConfig);
    coralHopperMotor.configure(coralHopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralAlgaeWristMotor.configure(coralAlgaeWristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralDetector.getConfigurator().apply(coralDetectorConfig);

    //Setting booleans to false at the beginning
    doWeHaveAlgae = false;
    isThereCoral = false;
  }

  //Setting the angle of the Pivot with Position Control
  public void setWristPivot(double position){
    wristPIDController.setReference(position, ControlType.kPosition);
  }

  //Setting the Wrist Motor with a speed percentage
  public void setWristSpeed(double speed){
    coralAlgaeWristMotor.set(speed);

  }

  //Getting the Angle of the Wrist
  public double getWristPosition(){
    return pivotAbsEncoder.getPosition();
  }

  //Checking to see if the CANRange is detecting a coral in the mechanism
  public boolean getCoralDetection(){
    return coralDetector.getIsDetected().getValue();
  }

  //Setting the CXAMotor to a velocity for quick feeding, coral scoring, Algae removal, and Algae Scoring 
  public void setCXAVelocity(double vel){
    cXAMotor.setControl(velocity.withSlot(0).withVelocity(vel));
  }

  public void setCXAFastVelocity(double vel){
    cXAMotor.setControl(velocity.withSlot(1).withVelocity(vel));
  }

  //Setting the CXA Motor to a speed percentage
  public void setCXASpeed(double speed){
    cXAMotor.set(speed);
  }

  //Setting the Coral Hopper Motor to a speed percentage
  public void setCoralHopperMotorSpeed(double speed){
    coralHopperMotor.set(speed);
  }

  //Getting the CXA Motor Current to automatically tell the robot when the Algae is in the Wrist End Effector
  public double getCXAMotorCurrent(){
    return cXAMotor.getTorqueCurrent(true).getValueAsDouble();
  }

  //Setting the boolean for if we have algae to true to hold the algae in our wrist
  public void setAlgaeBool(boolean ag){
    doWeHaveAlgae = ag;
  }

  //Checking to see if we do have an Algae
  public boolean doWeHaveAlgae(){
    return doWeHaveAlgae;
  }

  public void setCoralInside(boolean coralIsThere){
    isThereCoral = coralIsThere;
  }
  public boolean isThereCoral(){
    return isThereCoral;
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
    SmartDashboard.putBoolean("Is Coral Detected: ", getCoralDetection());
    SmartDashboard.putNumber("Wrist Angle", getWristPosition());
    SmartDashboard.putNumber("CXA Motor Current: ", getCXAMotorCurrent());
    SmartDashboard.putBoolean("Coral is In Place", isThereCoral());
    // System.out.println("CXA Motor Current Limit" + cXAMotor.getTorqueCurrent(true).getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
