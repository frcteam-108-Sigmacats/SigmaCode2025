// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralXAlgaeWristConstants;
import frc.robot.Constants.Elevatorconstants;
import frc.robot.subsystems.CoralXAlgaeMech;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorRestCommand extends Command {
  private Elevator elevatorSub;
  private CoralXAlgaeMech cxaMech;
  private Vision vision;
  private boolean elevatorAtL4;
  private int counter;
  /** Creates a new ElevatorRestCommand. */
  public ElevatorRestCommand(Elevator elevatorSub, CoralXAlgaeMech cxaMech, Vision vision) {
    this.elevatorSub = elevatorSub;
    this.cxaMech = cxaMech;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    if(Math.abs(Elevatorconstants.L4 - elevatorSub.getElevatorPosition()) <= 0.2){
      elevatorAtL4 = true;
    }
    else{
      elevatorAtL4 = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(CoralXAlgaeWristConstants.restWristPosition - cxaMech.getWristPosition()) <= 11 || cxaMech.getWristPosition() >= 355){
      if(vision.istheretagLeftLL() || vision.istheretagRightLL()){
        if((vision.getLeftLLTA() > 0 && vision.getLeftLLTA() < 9.5) || (vision.getRightLLTA() > 0 && vision.getRightLLTA() < 9.5)){
          elevatorSub.setElevatorPosition(0);
        }
      }
      else{
        elevatorSub.setElevatorPosition(0);
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
