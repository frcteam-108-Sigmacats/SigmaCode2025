// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CXAWristCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralXAlgaeMech;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestWristPivot extends Command {
  private CoralXAlgaeMech cXASub;
  private double position;
  /** Creates a new TestWristPivot. */
  public TestWristPivot(CoralXAlgaeMech cXASub, double position) {
    this.cXASub = cXASub;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cXASub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cXASub.setWristPivot(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cXASub.setWristPivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
