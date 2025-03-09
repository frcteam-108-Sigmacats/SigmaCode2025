// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Elevatorconstants;
import frc.robot.subsystems.CoralXAlgaeMech;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPositionReef extends Command {
  private Elevator elevatorSub;
  private CoralXAlgaeMech cxaMech;
  private String level;
  /** Creates a new SetElevatorPositionReef. */
  public SetElevatorPositionReef(Elevator elevatorSub, CoralXAlgaeMech cxaMech, String level) {
    this.elevatorSub = elevatorSub;
    this.level = level;
    this.cxaMech = cxaMech;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(cxaMech.doWeHaveAlgae()){
      level = "Net";
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(level){
      case "L1":
      if(!cxaMech.getCoralDetection()){
        elevatorSub.setElevatorPosition(Elevatorconstants.L1);
      }
        break;
      case "L2":
        if(!cxaMech.getCoralDetection()){
          elevatorSub.setElevatorPosition(Elevatorconstants.L2);
        }
        break;
      case "L3":
        if(!cxaMech.getCoralDetection()){
          elevatorSub.setElevatorPosition(Elevatorconstants.L3);
        }
        break;
      case "L4":
        if(!cxaMech.getCoralDetection()){
          elevatorSub.setElevatorPosition(Elevatorconstants.L4);
        }
        break;
      case "A1":
        elevatorSub.setElevatorPosition(Elevatorconstants.A1);
        break;
      case "A2":
        elevatorSub.setElevatorPosition(Elevatorconstants.A2);
        break;
      case "Net":
        elevatorSub.setElevatorPosition(5.4);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
