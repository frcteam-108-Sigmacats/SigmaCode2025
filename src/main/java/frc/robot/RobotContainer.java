// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorRestCommand;
import frc.robot.commands.SetReefLevel;
import frc.robot.commands.SwervePoseGenerator;
import frc.robot.commands.WristRestCommand;
import frc.robot.commands.HumanStationFeeder;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Swervemodule;
import frc.robot.subsystems.Vision;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final Vision vision = new Vision();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private Trigger bX, bB, bA, bY, LT, RT, LB, RB, upPov;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    swerveDrive.setDefaultCommand(new Drive(swerveDrive, m_driverController, fieldRelative));
    algaeSub.setDefaultCommand(new RestAlgaeIntake(algaeSub, AlgaeIntakeConstants.algaeRestPivotPosition, AlgaeIntakeConstants.algaeRestSpeed));
    cXASub.setDefaultCommand(new WristRestCommand(cXASub, elevatorSub));
    elevatorSub.setDefaultCommand(new ElevatorRestCommand(elevatorSub, cXASub));
    // cXASub.setDefaultCommand(getAutonomousCommand());
    // Configure the trigger bindings
    configureBindings();

   // bX.whileTrue(new RunAlgaeIntake(algaeSub, AlgaeIntakeConstants.algaeIntakePivotPosition, AlgaeIntakeConstants.algaeIntakeSpeed));//42
   // bB.whileTrue(new RunAlgaeOuttake(algaeSub, AlgaeIntakeConstants.algaeOuttakePivotPosition, AlgaeIntakeConstants.algaeOuttakeSpeed));
        // Configure the trigger bindings
    bA.whileTrue(new ReefScore(elevatorSub, cXASub, false, 1));
    bA.whileFalse(new ReefScore(elevatorSub, cXASub, true, 1));
    bB.whileTrue(new ReefScore(elevatorSub, cXASub, false, 2));
    bB.whileFalse(new ReefScore(elevatorSub, cXASub, true, 2));
    bY.whileTrue(new ReefScore(elevatorSub, cXASub, false, 3));
    bY.whileFalse(new ReefScore(elevatorSub, cXASub, true, 3));
    bX.whileTrue(new ReefScore(elevatorSub, cXASub, false, 4));
    bX.whileFalse(new ReefScore(elevatorSub, cXASub, true, 4));
    // LT.whileTrue(new ReefScore(elevatorSub, cXASub, false));
    // LT.whileFalse(new ReefScore(elevatorSub, cXASub, true));
    // RT.whileTrue(new HumanStationFeeder(cXASub, elevatorSub));
    RT.toggleOnTrue(new HumanStationFeeder(cXASub, elevatorSub));
    upPov.onTrue(new InstantCommand(()-> swerveDrive.zeroHeading(DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.kPi : Rotation2d.kZero)));
    LB.whileTrue(new SwervePoseGenerator(swerveDrive, vision, true));
    RB.whileTrue(new SwervePoseGenerator(swerveDrive, vision, false));

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
   RT = m_driverController.rightTrigger();
   LB = m_driverController.leftBumper();
   RB = m_driverController.rightBumper();
   upPov = m_driverController.povUp();
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

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
