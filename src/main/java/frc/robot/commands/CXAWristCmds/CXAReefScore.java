// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CXAWristCmds;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralXAlgaeWristConstants;
import frc.robot.Constants.Elevatorconstants;
import frc.robot.subsystems.CoralXAlgaeMech;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CXAReefScore extends Command {
  private CoralXAlgaeMech cxaMech;
  private Elevator elevatorSub;
  private boolean readyToExecute;
  private int counter;
  private boolean isFinished;
  private String level;
  /** Creates a new CXAReefScore. */
  public CXAReefScore(CoralXAlgaeMech cxaMech, Elevator elevatorSub, boolean readyToExecute, String level) {
    this.cxaMech = cxaMech;
    this.elevatorSub = elevatorSub;
    this.readyToExecute = readyToExecute;
    this.level = level;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cxaMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(level == "L4"){
      if(Math.abs(Elevatorconstants.L4 - elevatorSub.getElevatorPosition()) <= 0.1)
      cxaMech.setWristPivot(CoralXAlgaeWristConstants.l4WristPosition);
    }
    else{
      cxaMech.setWristPivot(CoralXAlgaeWristConstants.restWristPosition);
    }
    if(readyToExecute){
      cxaMech.setCXAVelocity(CoralXAlgaeWristConstants.cxaMotorFeedVelocity);
      counter++;
      System.out.println("Counter: "+ counter );
    }
    else{
      cxaMech.setCXAVelocity(0);
    }
    if(counter >= 25){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
