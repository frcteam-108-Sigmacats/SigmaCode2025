// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControllerCmds;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.CXAWristCmds.CXAWristNetScore;
import frc.robot.commands.ElevatorCommands.SetElevatorPositionReef;
import frc.robot.subsystems.CoralXAlgaeMech;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NetScore extends ParallelCommandGroup {
  /** Creates a new NetScore. */
  public NetScore(CoralXAlgaeMech cxaMech, Elevator elevatorSub, String level, boolean readyToExecute) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new CXAWristNetScore(cxaMech, readyToExecute), new SetElevatorPositionReef(elevatorSub, level));
  }
}
