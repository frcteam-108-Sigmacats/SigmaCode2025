// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControllerCmds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PrimeClimb extends Command {
  private Climber climberMech;
  private SwerveDrive swerveSub;
  private CommandXboxController driverController;
  private int counter;
  private int climbStates;
  /** Creates a new PrimeClimb. */
  public PrimeClimb(Climber climberMech, SwerveDrive swerveSub, CommandXboxController driverController) {
    this.climberMech = climberMech;
    this.swerveSub = swerveSub;
    this.driverController = driverController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    climbStates = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(climbStates){
      case 0:
        swerveSub.setSlowSpeedBool(true);
        counter++;
        climberMech.setServoPosition(0.5);
        if(counter >= 15){
          climbStates = 1;
        }
        break;
      case 1:
        counter = 0;
        climberMech.setClimberPosition(ClimberConstants.climberOutPosition);
        if(climberMech.getClimbPosition() >= (ClimberConstants.climberOutPosition - 3) && climberMech.getClimbPosition() <= 350){
          climbStates = 2;
        }
        break;
      case 2:
        counter++;
        climberMech.setServoPosition(0);
        climberMech.stopClimberWinchMotor();
        climberMech.setClimberIntakeSpeed(ClimberConstants.climerIntakeSpeed);
        if(climberMech.isCageIn() && counter >= 15){
          climbStates = 3;
        }
        break;
      case 3:
        climberMech.setClimberIntakeSpeed(ClimberConstants.climerIntakeSpeed);
        climberMech.setClimberPosition(ClimberConstants.climberInPosition);
        if(climberMech.getClimbPosition() <= (ClimberConstants.climberInPosition + 3)){
          climbStates = 4;
        }
        break;
      case 4:
        climberMech.setClimberIntakeSpeed(0.0);
          if(driverController.povDown().getAsBoolean()){
            climberMech.setClimberWinchSpeed(ClimberConstants.climbInSpeed);
          }
          else{
            climberMech.stopClimberWinchMotor();
          }
          break;
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
