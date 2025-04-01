// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunAlgaeIntake extends Command {
  private AlgaeIntake algaeSub;
  private double position;
  private double speed;
  private int counter;
  /** Creates a new RunAlgaeIntake. */
  private boolean isFinished;
  public RunAlgaeIntake(AlgaeIntake algaeSub, double position, double speed) {
    this.algaeSub = algaeSub;
    this.position = position;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeSub.setAlgaePivot(position);
    algaeSub.setAlgaeRollerSpeed(speed);
      if(algaeSub.getAlgaeRollerCurrent() > 30){
        algaeSub.setAlgaeBool(true);
        counter++;
      }
      if(counter>= 50){
        isFinished = true;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
