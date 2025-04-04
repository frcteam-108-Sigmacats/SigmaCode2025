// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.VisionConstants;

public class SwerveDrive extends SubsystemBase {
  
  //Instantiating our 4 Swerve Modules on the Robot with its CAN IDs and Absolute Encoder Angle Offsets in Radians
  public final Swervemodule fLeftModule = new Swervemodule(SwerveDriveConstants.fLDriveMotorID,SwerveDriveConstants.fLTurnMotorID, SwerveDriveConstants.fLAbsEncoderOffset);

  public final Swervemodule fRightModule = new Swervemodule(SwerveDriveConstants.fRDriveMotorID,SwerveDriveConstants.fRTurnMotorID, SwerveDriveConstants.fRAbsEncoderOffset);

  public final Swervemodule bLeftModule = new Swervemodule(SwerveDriveConstants.bLDriveMotorID,SwerveDriveConstants.bLTurnMotorID, SwerveDriveConstants.bLAbsEncoderOffset);

  public final Swervemodule bRightModule = new Swervemodule(SwerveDriveConstants.bRDriveMotorID,SwerveDriveConstants.bRTurnMotorID, SwerveDriveConstants.bRAbsEncoderOffset);

  //Instantiating a variable to check if we are Blue Alliance or not
  private boolean isBlueAlliance;

  //Instantiating our Gyro to track our Robot Angle and telling it which CANBus it is on
  private Pigeon2 gyro = new Pigeon2(1, "*");

  //Making a Field Image to see where our robot think it is on the Field
  private Field2d field = new Field2d();

  //Instantiating a Vision object to add our vision 
  private Vision vision = new Vision();

  //Instantiating our Swerve Pose Estimator
  private static SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  
  //Adding our 4 modules into an array
  private Swervemodule [] modules = {fLeftModule, fRightModule, bLeftModule, bRightModule};

  private boolean snapToHumanStation = false;

  private boolean slowSpeed = false;


  // private SlewRateLimiter driveLimiter = new SlewRateLimiter(70);
  // private SlewRateLimiter turnLimiter= new SlewRateLimiter(70);
  // private SlewRateLimiter rotLimiter = new SlewRateLimiter(10);

  private Pose2d targetPose = new Pose2d();
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    //Restoring our gyro to factory default
    gyro.getConfigurator().apply(new Pigeon2Configuration());
 
    //Clearing any sticky faults our gyro has during boot up
    gyro.clearStickyFault_BootDuringEnable();

    //Assigning our Pose Estimator with our Swerve Kinematic, gyro heading, module positions, and a empty pose
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.swerveKinematics, getHeading(), getModulePosition(), new Pose2d());

    //Checking to see which alliance we are on
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      isBlueAlliance = true;
    }
    else{
      isBlueAlliance = false;
    }

    //Configuring our Path Planner for Auto and Path Finder 
    try{
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
      this::getPose, //Getting our Robot Pose
      this::resetEstimator, //Reseting our Pose for Autonomous
      this::getSpeeds, //Getting our Module Speeds
      this::driveRobotRelative, //Basically how path planner will assign speed values to our module to run a path
      new PPHolonomicDriveController(new PIDConstants(5.0,0.0,0.0), new PIDConstants(5.0,0.0,0.0)), //PID Constants to follow our path accurately 
      config, //Our Robot Configurations on Path Planner
      () -> {//Checking to see which alliuance we are on to flip our auto's path
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
       this);//Adding our subsystem dependency to run Auto 
    }
    catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    //Adds our Field to Smart Dashboard
    SmartDashboard.putData(field);
  }
    public void setHumanStationBoolean(boolean humanStationSnap){
      snapToHumanStation = humanStationSnap;
    }
    public boolean getHumanStationSnap(){
      return snapToHumanStation;
    }

  //Get our Gyro Heading  between 0 and 360 degrees
    public Rotation2d getHeading(){
      return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));
    }

    //Gets our Robot Pose from our Pose Estimator
    public Pose2d getPose(){
      return swerveDrivePoseEstimator.getEstimatedPosition();
    }
    
    //Gets our Robot Pose from our Pose Estimator for Vision class to set up our Robot Orientation
    public static Pose2d getPoseForVision(){
      return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    //Resetting our Pose Estimators Heading
    public void zeroHeading(Rotation2d heading){
      swerveDrivePoseEstimator.resetPosition(getHeading(), getModulePosition(), new Pose2d(getPose().getTranslation(), heading));
    }

    //Our Drive Function where we get our translation and rotation values and see if we want to drive field centric or robot centric
    public void drive(Translation2d translation, double rotation, boolean fieldRelative){
      SwerveModuleState[] swerveModuleStates =
          SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(
              fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                  translation.getX(), 
                                  translation.getY(), 
                                  rotation, 
                                  getYaw().minus(getAllianceColor() ? Rotation2d.kPi : Rotation2d.kPi)
                              )
                              : new ChassisSpeeds(
                                  translation.getX(), 
                                  translation.getY(), 
                                  rotation)
                              );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveConstants.kMaxSpeedMPS);//Makes sure our robot never goes beyond the max speeds

        //Setting our Module Desired States (basically direction the wheels should face with our desired speed)
        fLeftModule.setDesiredState(swerveModuleStates[0]);
        fRightModule.setDesiredState(swerveModuleStates[1]);
        bLeftModule.setDesiredState(swerveModuleStates[2]);
        bRightModule.setDesiredState(swerveModuleStates[3]);
    }

    //Setting our Module States
    public void setModuleStates(SwerveModuleState[] desiredStates){
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.kMaxSpeedMPS);
        fLeftModule.setDesiredState(desiredStates[0]);
        fRightModule.setDesiredState(desiredStates[1]);
        bLeftModule.setDesiredState(desiredStates[2]);
        bRightModule.setDesiredState(desiredStates[3]);
    }

    //Getting our Module States
     public SwerveModuleState[] getModuleStates(){
      SwerveModuleState[] states = new SwerveModuleState[4];
      for(int i = 0; i < states.length; i++){
        states[i] = modules[i].getState();
      }
      return states;
      
    }

    //Seeing if we are Blue Alliance or not
    public boolean getAllianceColor(){
      return isBlueAlliance;
    }
    // public Command SwervePoseGeneration(boolean left){
    //   Pose2d targetPose = new Pose2d();
    //   if(isBlueAlliance){
    //     targetPose = left? VisionConstants.leftBluePoses.get((int) vision.getLeftLLTagID()): VisionConstants.rightBluePoses.get((int) vision.getLeftLLTagID());
    //   }
    //   else{
    //     targetPose = left? VisionConstants.leftRedPoses.get((int) vision.getLeftLLTagID()): VisionConstants.rightRedPoses.get((int) vision.getLeftLLTagID());

    //   }
    //   // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
    //   //   getPose(),
    //   //   targetPose
    //   // );
    //   // PathConstraints constraints = new PathConstraints(3.0, 4.0, 2 * Math.PI, 4 * Math.PI);
    //   // PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, 
    //   // new GoalEndState(0, targetPose.getRotation()));
    //   // return runOnce(()-> 
    //   // AutoBuilder.followPath(path));
    //   PathConstraints constraints = new PathConstraints(6.0, 7.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
    //   Command pathFindCommand = AutoBuilder.pathfindToPose(targetPose, constraints);
    //   return pathFindCommand;
    // }

    //Gets the modules position (How much did the robot drive forward or backwards and left or right in meters and the direction the wheels are facing)
    public SwerveModulePosition[] getModulePosition(){
      SwerveModulePosition[] positions = new SwerveModulePosition[4];
      for(int i = 0; i < positions.length; i++){
        positions[i] = modules[i].getModulePosition();
      }
      return positions;
    }

    //Setting our Target Pose for our Path Generator Command
    public void setTargetPose(Pose2d tgPose){
      targetPose = tgPose;
    }

    //Getting our Target Pose for our Path Generator Command
    public Pose2d getTargetPose(){
      return targetPose;
    }

    //Resetting the Pose Estimator
      public void resetEstimator(Pose2d pose){
        swerveDrivePoseEstimator.resetPosition(getHeading(), getModulePosition(), pose);
      }

    //Sets the drive encoders to 0
    public void resetEncoders(){
      fLeftModule.resetEncoders();
      fRightModule.resetEncoders();
      bLeftModule.resetEncoders();
      bRightModule.resetEncoders();
    }

  //Same thing as getting the heading but is not restricted to 360 degrees
  public Rotation2d getYaw(){
    double yaw = gyro.getYaw().getValueAsDouble();
    return (SwerveDriveConstants.gyroReversed) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
  }

  //Getting our Swerve Chassis Speeds used for AUTO
  public ChassisSpeeds getSpeeds(){
    SwerveModuleState[] states = getModuleStates();
    return SwerveDriveConstants.swerveKinematics.toChassisSpeeds(states);
  }

  //Having our robot drive field relative used for AUTO
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds){
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  //Having our robot drive robot centric used for AUTO
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);

    setModuleStates(targetStates);
    
  }

  public void setSlowSpeedBool(boolean slowSpeed){
    this.slowSpeed = slowSpeed;
  }
  public boolean getSlowSpeed(){
    return slowSpeed;
  }

    
    

  

  @Override
  public void periodic() {
    swerveDrivePoseEstimator.update(getHeading(), getModulePosition());
    field.setRobotPose(getPose());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Yaw: ", getYaw().minus(getAllianceColor() ? Rotation2d.kPi: Rotation2d.kZero).getDegrees());
    SmartDashboard.putNumber("Robot Pose X: ", getPose().getX());
    SmartDashboard.putNumber("Robot Pose Y: ", getPose().getY());
    SmartDashboard.putNumber("Robot Pose Rotation: ", getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Robot Target Pose X: ", getTargetPose().getX());
    SmartDashboard.putNumber("Robot Target Pose Y: ", getTargetPose().getY());
    SmartDashboard.putNumber("Robot Target Pose Rotation: ", getTargetPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Front Left Drive Output: ", fLeftModule.getMotorVoltage());
    SmartDashboard.putNumber("Front Right Drive Output: ", fRightModule.getMotorVoltage());
    SmartDashboard.putNumber("Back Left Drive Output: ", bLeftModule.getMotorVoltage());
    SmartDashboard.putNumber("Back Right Drive Output: ", bRightModule.getMotorVoltage());

    //If either limelights see an apriltag and our robot is not spinning so fast we update our position from camera POV
    if(vision.istheretagLeftLL() && gyro.getAngularVelocityZDevice().getValueAsDouble() < 720){
      swerveDrivePoseEstimator.addVisionMeasurement(vision.getLeftLLBotPose().pose, vision.getLeftLLBotPose().timestampSeconds);
    }
    if(vision.istheretagRightLL() && gyro.getAngularVelocityZDevice().getValueAsDouble() < 720){
      swerveDrivePoseEstimator.addVisionMeasurement(vision.getRightLLBotPose().pose, vision.getRightLLBotPose().timestampSeconds);
    }
  }
}

