// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Swervemodule;
import frc.robot.subsystems.Vision;
import frc.robot.commands.HumanStationFeederAuto;
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
import frc.robot.commands.CXAWristCmds.WristRestCommand;
import frc.robot.commands.ControllerCmds.AlgaeRemovalCommand;
import frc.robot.commands.ControllerCmds.DeepClimb;
import frc.robot.commands.ControllerCmds.Drive;
import frc.robot.commands.ControllerCmds.HumanStationFeeder;
import frc.robot.commands.ControllerCmds.PrimeClimb;
import frc.robot.commands.ControllerCmds.ReefScore;
import frc.robot.commands.ControllerCmds.SwervePoseGenerator;
import frc.robot.commands.ElevatorCommands.ElevatorRestCommand;
import frc.robot.commands.ElevatorCommands.SetReefLevel;
import frc.robot.subsystems.CoralXAlgaeMech;
import frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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

  private Trigger bX, bB, bA, bY, LT, RT, LB, RB, upPov, downPov, rightPov, backLeftPaddle, backRightPaddle;

  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    swerveDrive.setDefaultCommand(new Drive(swerveDrive, elevatorSub, m_driverController, fieldRelative));
    algaeSub.setDefaultCommand(new RestAlgaeIntake(algaeSub, AlgaeIntakeConstants.algaeRestPivotPosition, AlgaeIntakeConstants.algaeRestSpeed));
    cXASub.setDefaultCommand(new WristRestCommand(cXASub, elevatorSub));
    elevatorSub.setDefaultCommand(new ElevatorRestCommand(elevatorSub, cXASub, vision));

    // cXASub.setDefaultCommand(getAutonomousCommand());
    // Configure the trigger bindings
    configureBindings();
    makeAuto();

   // bX.whileTrue(new RunAlgaeIntake(algaeSub, AlgaeIntakeConstants.algaeIntakePivotPosition, AlgaeIntakeConstants.algaeIntakeSpeed));//42
   // bB.whileTrue(new RunAlgaeOuttake(algaeSub, AlgaeIntakeConstants.algaeOuttakePivotPosition, AlgaeIntakeConstants.algaeOuttakeSpeed));
        // Configure the trigger bindings
    // LT.whileTrue(new ReefScore(elevatorSub, cXASub, false));
    // LT.whileFalse(new ReefScore(elevatorSub, cXASub, true));
    // RT.whileTrue(new HumanStationFeeder(cXASub, elevatorSub));
    // upPov.onTrue(new InstantCommand(()-> swerveDrive.zeroHeading(DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.kPi : Rotation2d.kZero)));
    // upPov.whileTrue(new RunAlgaeIntake(algaeSub, AlgaeIntakeConstants.algaeIntakePivotPosition, AlgaeIntakeConstants.algaeIntakeSpeed));
    // downPov.whileTrue(new RunAlgaeOuttake(algaeSub, AlgaeIntakeConstants.algaeOuttakePivotPosition, AlgaeIntakeConstants.algaeOuttakeSpeed));
    bY.whileTrue(new ReefScore(elevatorSub, cXASub, false, "L4"));
    bY.whileFalse(new ReefScore(elevatorSub, cXASub, true, "L4"));
    bB.whileTrue(new ReefScore(elevatorSub, cXASub, false, "L3"));
    bB.whileFalse(new ReefScore(elevatorSub, cXASub, true, "L3"));
    bA.whileTrue(new ReefScore(elevatorSub, cXASub, false, "L2"));
    bA.whileFalse(new ReefScore(elevatorSub, cXASub, true, "L2"));
    bX.whileTrue(new ReefScore(elevatorSub, cXASub, false, "L1"));
    bX.whileFalse(new ReefScore(elevatorSub, cXASub, true, "L1"));
    backRightPaddle.whileTrue(new AlgaeRemovalCommand(elevatorSub, cXASub, "A2"));
    backLeftPaddle.whileTrue(new AlgaeRemovalCommand(elevatorSub, cXASub, "A1"));
    LB.whileTrue(new SwervePoseGenerator(swerveDrive, vision, true));
    RB.whileTrue(new SwervePoseGenerator(swerveDrive, vision, false));
    LT.toggleOnTrue(new HumanStationFeeder(cXASub, elevatorSub));
    RT.whileTrue(new RunAlgaeIntake(algaeSub, AlgaeIntakeConstants.algaeIntakePivotPosition, AlgaeIntakeConstants.algaeIntakeSpeed));
    downPov.toggleOnTrue(new PrimeClimb(algaeSub));
    upPov.whileTrue(new DeepClimb(algaeSub, -0.3, -0.6));
    upPov.whileFalse(new DeepClimb(algaeSub, 0, 0));
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
   backLeftPaddle = m_driverController.leftStick();
   backRightPaddle = m_driverController.rightStick();
   upPov = m_driverController.povUp();
   downPov = m_driverController.povDown();
   rightPov = m_driverController.povRight();
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  }

  public void makeAuto(){
    NamedCommands.registerCommand("LeftAutoAlign", new SwervePoseGenerator(swerveDrive, vision, true));
    NamedCommands.registerCommand("RightAutoAlign", new SwervePoseGenerator(swerveDrive, vision, false));
    NamedCommands.registerCommand("HumanFeeder", new HumanStationFeederAuto(cXASub, elevatorSub));
    NamedCommands.registerCommand("L4Position", new ReefScore(elevatorSub, cXASub, false, "L4"));
    NamedCommands.registerCommand("L4Score", new ReefScore(elevatorSub, cXASub, true, "L4"));
    NamedCommands.registerCommand("RestElevator", new ElevatorRestCommand(elevatorSub, cXASub, vision));
    NamedCommands.registerCommand("RestWrist", new WristRestCommand(cXASub, elevatorSub));

    Command blueCageSAuto = AutoBuilder.buildAuto("CageSAuto");
    Command blueProcessorSAuto = AutoBuilder.buildAuto("ProcessorSAuto");

    // autoChooser.setDefaultOption("Nothing", null);
    // autoChooser.addOption("BlueCageSAuto", blueCageSAuto);
    // autoChooser.addOption("BlueProcessorSAuto", blueProcessorSAuto);
    autoChooser.setDefaultOption("Nothing", null);
    autoChooser.addOption("CageAuto", blueCageSAuto);
    autoChooser.addOption("ProcessorAuto", blueProcessorSAuto);

    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
