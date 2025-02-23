// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  private final SwerveDrive swerve;

private Translation2d translation;

  private double rotation;

  private boolean fieldRelative;

  private SwerveDrive swerveDrive;
  private CommandXboxController m_driverController;
  

  private SlewRateLimiter yLim = new SlewRateLimiter(20);
  private SlewRateLimiter xLim = new SlewRateLimiter(20);
  private SlewRateLimiter rotLim = new SlewRateLimiter(1);


  /** Creates a new Drive. */
  public Drive(SwerveDrive swerve, CommandXboxController driver, boolean fieldRelative) {
    this.swerve = swerve;
    m_driverController = driver;
    this.fieldRelative = fieldRelative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yAxis = m_driverController.getLeftY();
    double xAxis = m_driverController.getLeftX();
    double rotAxis = -m_driverController.getRightX();
// y=0 x=0.0//
    yAxis = (Math.abs(yAxis) < SwerveDriveConstants.deadband ? 0.0 : (yAxis * 2.5));
    xAxis = (Math.abs(xAxis) < SwerveDriveConstants.deadband ? 0.0 : (xAxis * 2.5));
    rotAxis = (Math.abs(rotAxis) < SwerveDriveConstants.deadband ? 0 : (rotAxis * 10));
    yAxis = yLim.calculate(yAxis);
    xAxis = xLim.calculate(xAxis);

    translation = new Translation2d(yAxis, xAxis).times(SwerveDriveConstants.kMaxSpeedMPS);
    rotation = rotAxis * SwerveDriveConstants.maxAngularspeed;

    swerve.drive(translation, rotation, fieldRelative);
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
