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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntake extends SubsystemBase {
  private SparkMax algaePivotMotor;
  private SparkMax algaeRollerMotor;

  private SparkMaxConfig algaePivotMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig alageRollerMotorConfig = new SparkMaxConfig();

  private SparkClosedLoopController pivotPIDController;

  private AbsoluteEncoder pivotAbsEncoder;

  private boolean isThereAlgae;
  /** Creates a new ExampleSubsystem. */
  public AlgaeIntake() {
    algaePivotMotor = new SparkMax(AlgaeIntakeConstants.algaePivotMotorID, MotorType.kBrushless);
    algaeRollerMotor = new SparkMax(AlgaeIntakeConstants.algaeRollerMotorID, MotorType.kBrushless);
    
    algaePivotMotorConfig.idleMode(IdleMode.kBrake);
    algaePivotMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.algaePivotMotorCurrentLimit);

    algaePivotMotorConfig.absoluteEncoder.positionConversionFactor(360);
    algaePivotMotorConfig.absoluteEncoder.inverted(true);

    algaePivotMotorConfig.closedLoop.pid(AlgaeIntakeConstants.pivotP, AlgaeIntakeConstants.pivotI, AlgaeIntakeConstants.pivotD);
    algaePivotMotorConfig.closedLoop.velocityFF(1/5676);
    algaePivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    algaePivotMotorConfig.closedLoop.positionWrappingEnabled(true);
    algaePivotMotorConfig.closedLoop.positionWrappingMinInput(0);
    algaePivotMotorConfig.closedLoop.positionWrappingMaxInput(360);

    pivotPIDController = algaePivotMotor.getClosedLoopController();
    pivotAbsEncoder = algaePivotMotor.getAbsoluteEncoder();

    alageRollerMotorConfig.idleMode(IdleMode.kCoast);
    alageRollerMotorConfig.smartCurrentLimit(AlgaeIntakeConstants.algaeRollerMotorCurrentLimit);
    alageRollerMotorConfig.inverted(true);

    algaePivotMotor.configure(algaePivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeRollerMotor.configure(alageRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    isThereAlgae = false;
  }
  public void SetAlgaeIntakePivotSpeed(double speed){
    algaePivotMotor.set(speed);
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

  public void setAlgaeBool(boolean ag){
    isThereAlgae = ag;
  }

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
