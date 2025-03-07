// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private TalonFX cXAMotor;

  private SparkMax coralHopperMotor;

  private SparkFlex coralAlgaeWristMotor;

  private TalonFXConfiguration cXAMotorConfig = new TalonFXConfiguration();
  private VelocityVoltage velocity = new VelocityVoltage(0);

  private SparkMaxConfig coralHopperMotorConfig = new SparkMaxConfig();
  private SparkFlexConfig coralAlgaeWristMotorConfig = new SparkFlexConfig();

  private AbsoluteEncoder pivotAbsEncoder;

  private SparkClosedLoopController wristPIDController;

  private CANrange coralDetector;

  private CANrangeConfiguration coralDetectorConfig = new CANrangeConfiguration();

  private boolean doWeHaveAlgae;
  /** Creates a new ExampleSubsystem. */
  public CoralXAlgaeMech() {
    cXAMotor = new TalonFX(CoralXAlgaeWristConstants.cXAMotorID, "*");
    cXAMotor.getConfigurator().apply(new TalonFXConfiguration());

    coralHopperMotor = new SparkMax(CoralXAlgaeWristConstants.coralHopperMotorID, MotorType.kBrushless);
    coralAlgaeWristMotor = new SparkFlex(CoralXAlgaeWristConstants.coralXAlgaeWristID, MotorType.kBrushless);

    coralDetector = new CANrange(CoralXAlgaeWristConstants.coralDetectorID, "*");
    coralDetector.getConfigurator().apply(new CANrangeConfiguration());

    coralDetectorConfig.FovParams.FOVRangeY = 11;
    coralDetectorConfig.FovParams.FOVRangeX = 11;
    coralDetectorConfig.FovParams.FOVCenterY = 10;
    coralDetectorConfig.ProximityParams.ProximityThreshold = 0.12;

    cXAMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cXAMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    cXAMotorConfig.CurrentLimits.StatorCurrentLimit = CoralXAlgaeWristConstants.coralMotorCurrentLimit;
    cXAMotorConfig.Slot0.kP = CoralXAlgaeWristConstants.velocityP;
    cXAMotorConfig.Slot0.kI = CoralXAlgaeWristConstants.velocityI;
    cXAMotorConfig.Slot0.kD = CoralXAlgaeWristConstants.velocityD;

    coralHopperMotorConfig.idleMode(IdleMode.kCoast);
    coralHopperMotorConfig.smartCurrentLimit(CoralXAlgaeWristConstants.coralHopperMotorCurrentLimit);

    coralHopperMotorConfig.signals.absoluteEncoderPositionAlwaysOn(false);
    coralHopperMotorConfig.signals.absoluteEncoderVelocityAlwaysOn(false);
    coralHopperMotorConfig.signals.analogPositionAlwaysOn(false);
    coralHopperMotorConfig.signals.externalOrAltEncoderPositionAlwaysOn(false);
    coralHopperMotorConfig.signals.externalOrAltEncoderVelocityAlwaysOn(false);
    coralHopperMotorConfig.signals.iAccumulationAlwaysOn(false);
    coralHopperMotorConfig.signals.primaryEncoderPositionAlwaysOn(false);
    
    coralAlgaeWristMotorConfig.idleMode(IdleMode.kBrake);
    coralAlgaeWristMotorConfig.smartCurrentLimit(CoralXAlgaeWristConstants.coralAlgaeWristCurrentLimit);
    
    coralAlgaeWristMotorConfig.absoluteEncoder.positionConversionFactor(360);
    coralAlgaeWristMotorConfig.absoluteEncoder.inverted(true);
    
    // coralAlgaeWristMotorConfig.softLimit.forwardSoftLimitEnabled(true);
    // coralAlgaeWristMotorConfig.softLimit.reverseSoftLimitEnabled(true);

    // coralAlgaeWristMotorConfig.softLimit.forwardSoftLimit(CoralXAlgaeWristConstants.forwardSoftLimitWrist);
    // coralAlgaeWristMotorConfig.softLimit.reverseSoftLimit(CoralXAlgaeWristConstants.reverseSoftLimitWrist);

    coralAlgaeWristMotorConfig.closedLoop.pid(CoralXAlgaeWristConstants.pivotP, CoralXAlgaeWristConstants.pivotI, CoralXAlgaeWristConstants.pivotD);
    coralAlgaeWristMotorConfig.closedLoop.velocityFF(CoralXAlgaeWristConstants.pivotFF);
    coralAlgaeWristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    coralAlgaeWristMotorConfig.closedLoop.positionWrappingEnabled(true);
    coralAlgaeWristMotorConfig.closedLoop.positionWrappingInputRange(0, 360);
    

    wristPIDController = coralAlgaeWristMotor.getClosedLoopController();
    pivotAbsEncoder = coralAlgaeWristMotor.getAbsoluteEncoder();


    cXAMotor.getConfigurator().apply(cXAMotorConfig);
    coralHopperMotor.configure(coralHopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralAlgaeWristMotor.configure(coralAlgaeWristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralDetector.getConfigurator().apply(coralDetectorConfig);

    doWeHaveAlgae = false;
  }

  public void setWristPivot(double position){
    wristPIDController.setReference(position, ControlType.kPosition);
  }

  public void setWristSpeed(double speed){
    coralAlgaeWristMotor.set(speed);
  }

  public double getWristPosition(){
    return pivotAbsEncoder.getPosition();
  }

  public boolean getCoralDetection(){
    return coralDetector.getIsDetected().getValue();
  }

  public void setCXAVelocity(double vel){
    cXAMotor.setControl(velocity.withVelocity(vel));
  }

  public void setCXASpeed(double speed){
    cXAMotor.set(speed);
  }

  public void setCoralHopperMotorSpeed(double speed){
    coralHopperMotor.set(speed);
  }

  public double getCXAMotorCurrent(){
    return cXAMotor.getTorqueCurrent().getValueAsDouble();
  }

  public void setAlgaeBool(boolean ag){
    doWeHaveAlgae = ag;
  }
  public boolean doWeHaveAlgae(){
    return doWeHaveAlgae;
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
