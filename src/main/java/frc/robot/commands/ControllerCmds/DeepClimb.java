// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControllerCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeepClimb extends Command {
  private AlgaeIntake algaeIntake;
  private double intakePivotSpeed;
  private double climbSpeed;
  /** Creates a new DeepClimb. */
  public DeepClimb(AlgaeIntake algaeIntake, double intakePivotSpeed, double climbSpeed) {
    this.algaeIntake = algaeIntake;
    this.intakePivotSpeed = intakePivotSpeed;
    this.climbSpeed = climbSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeIntake.setClimbMotorSpeed(climbSpeed);
    algaeIntake.SetAlgaeIntakePivotSpeed(intakePivotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
