// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CXAWristCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralXAlgaeWristConstants;
import frc.robot.subsystems.CoralXAlgaeMech;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CXAWristHumanStationFeederAuto extends Command {
  private CoralXAlgaeMech cxaMech;
  private boolean isFinished;
  private boolean isCoralThere;
  private boolean startCounter; 
  private int counter;
  private boolean coralInside;
  /** Creates a new CXAWirstHumanStationFeederAuto. */
  public CXAWristHumanStationFeederAuto(CoralXAlgaeMech cxaMech) {
    this.cxaMech = cxaMech;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cxaMech);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralInside = cxaMech.isThereCoral();
    isCoralThere = false;
    isFinished = false;
    counter = 0;
    startCounter = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cxaMech.setWristPivot(358.5);
    if(!isCoralThere){
      cxaMech.setCoralHopperMotorSpeed(CoralXAlgaeWristConstants.coralHopperSpeed);
      cxaMech.setCXAVelocity(CoralXAlgaeWristConstants.cxaMotorFeedVelocity);
    }
    else{
      cxaMech.setCoralHopperMotorSpeed(0.4); 
      cxaMech.setCXAVelocity(-1500);
    }
    if(counter >= 15 && !cxaMech.getCoralDetection()){
      isFinished = true;
    }
    if(cxaMech.getCoralDetection()){
      isCoralThere = true;
    }
    startCounter = cxaMech.getCoralDetection();
    if(startCounter){
      counter++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cxaMech.setCXAVelocity(0);
    cxaMech.setCoralHopperMotorSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished){
      System.out.println("Auto Feeder Done");
    }
    return isFinished;
  }
}
