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
  private PIDController xpidController = new PIDController(0.03, 0, 0);
  private PIDController ypidController = new PIDController(0.1, 0, 0);

  private SwerveDrive swerveSub;
  private Vision visionSub;
  private Translation2d translation = new Translation2d();
  private boolean left;
  private Pose2d target;
  private double xVal, yVal;
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
    xpidController.setTolerance(0.1);
    ypidController.setTolerance(0.4);
    xVal = 0;
    yVal = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(left){
        xVal = -xpidController.calculate(visionSub.getRightLLTX(), -11);
        yVal = ypidController.calculate(visionSub.getRightLLTY());
      }
      else{
        xVal = xpidController.calculate(visionSub.getLeftLLTX(), 5);
        yVal = ypidController.calculate(visionSub.getLeftLLTY());
      }
      System.out.println("X Value " + xVal);
      System.out.println("Y Value " + yVal);
      translation = new Translation2d(xVal, yVal);
      swerveSub.drive(translation, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(xVal) <= 1 && Math.abs(yVal) <= 1){
      System.out.println("Finished PID");
      return true;
    }
    return false;
  }
}
