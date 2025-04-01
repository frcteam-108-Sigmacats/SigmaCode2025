// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.fasterxml.jackson.databind.Module.SetupContext;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;



public class Swervemodule extends SubsystemBase {

  //Instantiating our Drive and Turn Motors for our Swerve Module
  private TalonFX drivemotor;
  private SparkMax turnmotor;

  //Instantiating our Absolute Encoder to track our wheel's angle
  private AbsoluteEncoder absEncoder;

  //Instantiating our Configuation for configuring our drive motor
  private TalonFXConfiguration drirveMotorconfig = new TalonFXConfiguration();

  private VelocityVoltage velocity = new VelocityVoltage(0);

  //Instantiating our PID Controller for our Turn Motor
  private SparkClosedLoopController turnPIDcontroller;

  //Instantiating our Turn Motor Configuration to configure out Turn Motor
  private SparkMaxConfig turnmotorConfig = new SparkMaxConfig();

  //Instantiating a variable to hold our Angle Offset
  private double absAngleoffset;

  //Instantiating a desired state to keep track of our previous desired state
  private SwerveModuleState m_desiredstate = new SwerveModuleState();

  /** Creates a new ExampleSubsystem. */
  public Swervemodule(int drivemotorID, int turnmotorID, double angleOffset ) {
    //Assigning our Drive and Turn Motor their CAN IDs
    drivemotor = new TalonFX(drivemotorID, "*");
    turnmotor = new SparkMax(turnmotorID,MotorType.kBrushless);

    //Having our Drive Motor restart with factory default configs
    drivemotor.getConfigurator().apply(new TalonFXConfiguration());

    drirveMotorconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;//Setting our Neutral Mode to our Drive Motor
    turnmotorConfig.idleMode(IdleMode.kCoast);//Setting our Idle Mode to our Turn Motor

    drirveMotorconfig.CurrentLimits.StatorCurrentLimitEnable = true;//Enabling out Current Limit
    drirveMotorconfig.CurrentLimits.StatorCurrentLimit = SwerveDriveConstants.driveMotorCurrentLimit;//Setting our Current Limit
  
    drirveMotorconfig.Feedback.SensorToMechanismRatio = SwerveDriveConstants.kDrivingMotorReduction;//Putting our Gear ratio for our drive motor

    drirveMotorconfig.Slot0.kP = SwerveDriveConstants.drivemotorP;//Setting our P Value for our Velocity control of our wheel
    drirveMotorconfig.Slot0.kI = SwerveDriveConstants.drivemotorI;//Setting our I Value for our Velocity control of our wheel
    drirveMotorconfig.Slot0.kD = SwerveDriveConstants.drivemotorD;//Setting our D Value for our Velocity control of our wheel

    drirveMotorconfig.Slot0.kV = 1 / SwerveDriveConstants.krakenRPM; // helps tells the pid controller with grav into equ//
   
    turnmotorConfig.idleMode( IdleMode.kCoast);//Setting the Idle Mode of our Turn Motor
    turnmotorConfig.smartCurrentLimit(SwerveDriveConstants.turnmotorCurrentLimit);//Setting the Current Limit of our Turn Motor

    turnmotorConfig.absoluteEncoder.positionConversionFactor(SwerveDriveConstants.turningFactor);//Setting our Encoder Angle in Radians
    turnmotorConfig.absoluteEncoder.velocityConversionFactor(SwerveDriveConstants.turningFactor / 60.0);//Setting our encoder velocity
    turnmotorConfig.absoluteEncoder.inverted(true);//Inverting our encoder

    turnmotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);//Telling our PID Controller our Feedback sensor is an Absolute Encoder connected to our Turn Motor
    turnmotorConfig.closedLoop.pid(SwerveDriveConstants.turnmotorP,SwerveDriveConstants.turnmotorI,SwerveDriveConstants.turnmotorD);

    turnmotorConfig.closedLoop.outputRange(-1.0, 1.0);//Setting our Turn Motors Max Output for PID Control
    turnmotorConfig.closedLoop.positionWrappingEnabled(true);//Enabling PID Wrapping to get to our desired position in the fastest direction possible
    turnmotorConfig.closedLoop.positionWrappingInputRange(0, SwerveDriveConstants.turningFactor);

    //Assigning our Absolute Encoder variable to the one connected to our turn motor
    absEncoder = turnmotor.getAbsoluteEncoder();

    //Assigning our PID Controller to the one built-in our Turn Motor
    turnPIDcontroller  = turnmotor.getClosedLoopController();

    //Configuring our Drive and Turn Motor
    drivemotor.getConfigurator().apply(drirveMotorconfig);
    turnmotor.configure(turnmotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Assigning our encoder angle offset
    absAngleoffset = angleOffset;

    //Assigning our desired state angle to our wheels current angle
    m_desiredstate.angle = new Rotation2d(absEncoder.getPosition());

    //Setting our drive motor position to zero on boot up
    drivemotor.setPosition(0);
    
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

  //Getting our Module State using the Drive Motor Encoder Velocity and Turn Motor Abs Encoder Position
  public SwerveModuleState getState(){
    return new SwerveModuleState(Units.rotationsToRadians(drivemotor.getVelocity().getValueAsDouble()) * Units.inchesToMeters(3.0) * 0.5,
    new Rotation2d(absEncoder.getPosition() - absAngleoffset));
  }

  //Getting our Module Position using our Drive Motor Encoder Position and Turn Motor Abs Encoder Position
  public SwerveModulePosition getModulePosition(){
    return new SwerveModulePosition(Units.rotationsToRadians(drivemotor.getPosition().getValueAsDouble()) * Units.inchesToMeters(3.0) * 0.5,
   new Rotation2d(absEncoder.getPosition()- absAngleoffset));
  }

  //Setting our Desired state of our modules with MPS and Radians
   public void setDesiredState(SwerveModuleState desiredstate){
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredstate.speedMetersPerSecond;
    correctedDesiredState.angle = desiredstate.angle.plus(Rotation2d.fromRadians(absAngleoffset));

    correctedDesiredState.optimize(new Rotation2d(absEncoder.getPosition()));//Limits the rotation of the wheel to not go beyond 90 degrees to fastest desired state postioning 

    //Setting our Drive Motor Velocity from Speed MPS to RPM
    drivemotor.setControl(velocity.withVelocity(Units.radiansToRotations(correctedDesiredState.speedMetersPerSecond / (Units.inchesToMeters(3.0) * 0.5))));

    //Setting our Turn Motor Position using to desired Radian
    turnPIDcontroller.setReference(correctedDesiredState.angle.getRadians(),
    ControlType.kPosition);
    
   }

   //Getting the Drive Motor Voltage
   public double getMotorVoltage(){
    return drivemotor.getMotorVoltage().getValueAsDouble();
   }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(drivemotor.getSupplyVoltage());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

public void resetEncoders() {
    // TODO Auto-generated method stub
    drivemotor.setPosition(0);
}

}
