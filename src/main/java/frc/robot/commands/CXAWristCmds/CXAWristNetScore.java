// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CXAWristCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralXAlgaeWristConstants;
import frc.robot.subsystems.CoralXAlgaeMech;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CXAWristNetScore extends Command {
  private CoralXAlgaeMech cxaMech;
  private boolean readyToExecute;
  private boolean isFinished;
  private boolean isThereAlgae;
  private int counter;
  /** Creates a new CXAWristNetScore. */
  public CXAWristNetScore(CoralXAlgaeMech cxaMech, boolean readyToExecute) {
    this.cxaMech = cxaMech;
    this.readyToExecute = readyToExecute;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cxaMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    counter = 0;
    cxaMech.setCXAVelocity(-1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cxaMech.setWristPivot(0);
    if(readyToExecute){
      cxaMech.setCXAVelocity(3000);
      counter++;
    }
    else{
      cxaMech.setCXAVelocity(-1000);
    }

    if(counter >= 25){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
