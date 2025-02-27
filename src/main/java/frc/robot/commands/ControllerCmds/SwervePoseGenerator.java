// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControllerCmds;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetTargetPose;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwervePoseGenerator extends SequentialCommandGroup {
  /** Creates a new SwervePoseGenerator. */
  public SwervePoseGenerator(SwerveDrive swerveSub, Vision visionSub, boolean left) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetTargetPose(swerveSub, visionSub, left), 
      Commands.defer(() -> {
        return AutoBuilder.pathfindToPose(swerveSub.getTargetPose(), 
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)));
      }, Set.of(swerveSub)) );
  }
}
