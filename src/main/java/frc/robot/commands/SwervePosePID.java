// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwervePosePID extends Command {
  private ProfiledPIDController positionController = new ProfiledPIDController(1.0, 0, 0, new Constraints(4, 5));
  private SwerveDrive swerveSub;
  private Vision visionSub;
  private Translation2d translation = new Translation2d();
  private boolean left;
  /** Creates a new SwervePosePID. */
  public SwervePosePID(SwerveDrive swerveSub, Vision visionSub, boolean left) {
    this.swerveSub = swerveSub;
    this.visionSub = visionSub;
    this.left = left;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(left){
      translation = new Translation2d(positionController.calculate(visionSub.getLeftLLTargetBotPose()[0], -1), positionController.calculate(0));
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
