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
public class WristRestCommand extends Command {
  private CoralXAlgaeMech cxaMech;
  private Elevator elevatorSub;
  private int counter;
  /** Creates a new WristRestCommand. */
  public WristRestCommand(CoralXAlgaeMech cxaMech, Elevator elevatorSub) {
    this.cxaMech = cxaMech;
    this.elevatorSub = elevatorSub;
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
    // if(Math.abs(CoralXAlgaeWristConstants.l4WristPosition - cxaMech.getWristPosition()) <= 5){
    //   if(Math.abs(0 - elevatorSub.getElevatorPosition()) <= 0.2){
    //     cxaMech.setWristPivot(0);
    //   }
    // }
    // else{
    //   cxaMech.setWristPivot(0);
    // }
    cxaMech.setWristPivot(0);
    if(cxaMech.doWeHaveAlgae().getAsBoolean()){
      cxaMech.setCXAVelocity(-1000);
      if(Math.abs(cxaMech.getCXAMotorCurrent()) < 10){
        cxaMech.setAlgaeBool(false);
      }
    }
    else{
      cxaMech.setCXAVelocity(0);
    }
    cxaMech.setCoralHopperMotorSpeed(0);
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
