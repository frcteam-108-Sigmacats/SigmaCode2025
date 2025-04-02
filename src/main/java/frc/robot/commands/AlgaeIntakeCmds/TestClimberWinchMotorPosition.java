// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestClimberWinchMotorPosition extends Command {
private Climber climberMech;

private double position;
  /** Creates a new TestAlgaePivotPosition. */
  public TestClimberWinchMotorPosition(Climber climberMech, double position) {
    this.climberMech = climberMech;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(climberMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(position != 0){
      if(climberMech.getClimbPosition()<= (ClimberConstants.climberOutPosition - 3) || (climberMech.getClimbPosition() >= 350 &&  climberMech.getClimbPosition() <= 359)){
        climberMech.setClimberPosition(position);
      }
      else{
        climberMech.stopClimberWinchMotor();
      }
    }
    else{
      if(climberMech.getClimbPosition() >= ClimberConstants.climberStowPosition + 3 || (climberMech.getClimbPosition() >=350 && climberMech.getClimbPosition() <= 359.99)){
        climberMech.setClimberPosition(position);
      }
      else{
        climberMech.stopClimberWinchMotor();
      }
    }

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
