// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.SetReefLevel;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Swervemodule;
import frc.robot.commands.AlgaeIntakeCmds.RestAlgaeIntake;
import frc.robot.commands.AlgaeIntakeCmds.RunAlgaeIntake;
import frc.robot.commands.AlgaeIntakeCmds.RunAlgaeOuttake;
import frc.robot.commands.AlgaeIntakeCmds.TestAlgaePivotPosition;
import frc.robot.commands.AlgaeIntakeCmds.TestAlgaePivotSpeed;
import frc.robot.commands.AlgaeIntakeCmds.TestAlgaeRoller;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.commands.CXAWristCmds.TestCXAMotor;
import frc.robot.commands.CXAWristCmds.TestCoralHopper;
import frc.robot.commands.CXAWristCmds.TestWristPivot;
import frc.robot.commands.ControllerCmds.ReefScore;
import frc.robot.subsystems.CoralXAlgaeMech;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final AlgaeIntake algaeSub = new AlgaeIntake();
  private final CoralXAlgaeMech cXASub = new CoralXAlgaeMech();
  private final Elevator elevatorSub = new Elevator();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private Trigger bX, bB, bA, bY, LT;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    swerveDrive.setDefaultCommand(new Drive(swerveDrive, m_driverController, fieldRelative));
    algaeSub.setDefaultCommand(new RestAlgaeIntake(algaeSub, AlgaeIntakeConstants.algaeRestPivotPosition, AlgaeIntakeConstants.algaeRestSpeed));
    // Configure the trigger bindings
    configureBindings();

   // bX.whileTrue(new RunAlgaeIntake(algaeSub, AlgaeIntakeConstants.algaeIntakePivotPosition, AlgaeIntakeConstants.algaeIntakeSpeed));//42
   // bB.whileTrue(new RunAlgaeOuttake(algaeSub, AlgaeIntakeConstants.algaeOuttakePivotPosition, AlgaeIntakeConstants.algaeOuttakeSpeed));
        // Configure the trigger bindings
    configureBindings();
    bA.onTrue(new SetReefLevel(elevatorSub, 1));
    bB.onTrue(new SetReefLevel(elevatorSub, 2));
    bY.onTrue(new SetReefLevel(elevatorSub, 3));
    bX.onTrue(new SetReefLevel(elevatorSub, 4));
    // LT.whileTrue(new ReefScore(elevatorSub, cXASub, false));
    // LT.whileFalse(new ReefScore(elevatorSub, cXASub, true));
    /*bA.whileTrue(new TestWristPivot(cXASub, 0));
    bB.whileTrue(new TestCXAMotor(cXASub, 0));
    bY.whileTrue(new TestCoralHopper(cXASub, 0));*/
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   bA = m_driverController.a();
   bX = m_driverController.x();
   bB = m_driverController.b();
   bY = m_driverController.y();
   LT = m_driverController.leftTrigger();
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    bA = m_driverController.a();
    bB = m_driverController.b();
    bY = m_driverController.y();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
