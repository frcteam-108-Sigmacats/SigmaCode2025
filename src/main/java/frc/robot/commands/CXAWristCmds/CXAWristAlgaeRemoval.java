// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CXAWristCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralXAlgaeWristConstants;
import frc.robot.Constants.Elevatorconstants;
import frc.robot.subsystems.CoralXAlgaeMech;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CXAWristAlgaeRemoval extends Command {
  private CoralXAlgaeMech cxaMech;
  private Elevator elevator;
  private int counter;
  private String level;
  /** Creates a new CXAWristAlgaeRemoval. */
  public CXAWristAlgaeRemoval(CoralXAlgaeMech cxaMech, Elevator elevator, String level) {
    this.cxaMech = cxaMech;
    this.elevator = elevator;
    this.level = level;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cxaMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    cxaMech.setCXAVelocity(CoralXAlgaeWristConstants.cxaMotorAlgaeRemovalVelocity);
    if(level == "A1"){
      if(Math.abs(Elevatorconstants.A1 - elevator.getElevatorPosition()) <= 0.2){
        cxaMech.setWristPivot(CoralXAlgaeWristConstants.algaeRemovalWristPosition);
      }
    }
    else{
      if(Math.abs(Elevatorconstants.A2 - elevator.getElevatorPosition()) <= 0.2){
        cxaMech.setWristPivot(CoralXAlgaeWristConstants.algaeRemovalWristPosition);
      }
    }
    if(counter >= 25){
      if(Math.abs(cxaMech.getCXAMotorCurrent()) > 30){
        cxaMech.setAlgaeBool(true);
      }
      else{
        cxaMech.setAlgaeBool(false);
      }
    }

    if(cxaMech.doWeHaveAlgae()){
      cxaMech.setWristPivot(CoralXAlgaeWristConstants.algaeRestPosition);
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
