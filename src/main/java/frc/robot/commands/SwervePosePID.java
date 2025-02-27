// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwervePosePID extends Command {
  private PIDController pidController = new PIDController(1.0, 0, 0);
  private SwerveDrive swerveSub;
  private Vision visionSub;
  private Translation2d translation = new Translation2d();
  private boolean left;
  private Pose2d target;
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
    target = swerveSub.getTargetPose();
    pidController.setTolerance(0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double distance = Math.sqrt(Math.pow(target.getX() - swerveSub.getPose().getX(), 2) + Math.pow(target.getY() - swerveSub.getPose().getY(),2));
      double speed = pidController.calculate(distance);
      translation = new Translation2d(speed, speed);
      swerveSub.drive(translation, 0, false);
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
