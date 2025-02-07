// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntake extends SubsystemBase {
  private SparkMax algaePivotMotor;
  private SparkMax algaeRollerMotor;

  private SparkMaxConfig algaePivotMotorConfig;
  private SparkMaxConfig alageRollerMotorConfig;

  private SparkClosedLoopController pivotPIDController;

  private AbsoluteEncoder pivotAbsEncoder;
  /** Creates a new ExampleSubsystem. */
  public AlgaeIntake() {
    algaePivotMotor = new SparkMax(AlgaeIntakeConstants.algaePivotMotorID, MotorType.kBrushless);
    algaeRollerMotor = new SparkMax(AlgaeIntakeConstants.algaeRollerMotorID, MotorType.kBrushless);
    
    algaePivotMotorConfig.idleMode(IdleMode.kBrake);
    algaePivotMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.algaePivotMotorCurrentLimit);

    algaePivotMotorConfig.absoluteEncoder.positionConversionFactor(360);

    algaePivotMotorConfig.closedLoop.pid(AlgaeIntakeConstants.pivotP, AlgaeIntakeConstants.pivotI, AlgaeIntakeConstants.pivotD);
    algaePivotMotorConfig.closedLoop.velocityFF(1/5676);
    algaePivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    pivotPIDController = algaePivotMotor.getClosedLoopController();
    pivotAbsEncoder = algaePivotMotor.getAbsoluteEncoder();

    alageRollerMotorConfig.idleMode(IdleMode.kCoast);
    alageRollerMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.algaeRollerMotorCurrentLimit);

    algaePivotMotor.configure(algaePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeRollerMotor.configure(alageRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  public void setAlgaePivot(double position){
    pivotPIDController.setReference(position, ControlType.kPosition);
  }

  public double getAlgaePivotPosition(){
    return pivotAbsEncoder.getPosition();
  }

  public void setAlgaeRollerSpeed(double speed){
    algaeRollerMotor.set(speed);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
