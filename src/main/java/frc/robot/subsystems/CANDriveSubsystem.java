// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class CANDriveSubsystem extends SubsystemBase {
    private final SparkMax leftLeader = new SparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushless);
    // private final SparkMax leftFollower;
    private final SparkMax rightLeader = new SparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushless);
    // private final SparkMax rightFollower;
    private RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private RelativeEncoder rightEncoder = rightLeader.getEncoder(); 

    DifferentialDriveWheelSpeeds wheelSpeeds;
    
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    private final PIDController leftPID = new PIDController(.0004, 0, 0.0002);
    private final PIDController rightPID = new PIDController(.0004, 0, 0.0002);

    DCMotor neo = DCMotor.getNEO(1);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    0.001, // volts
    0.0001, // volt * seconds / meter
    0.001  // volt * seconds^2 / meter
);
    private final Field2d m_field = new Field2d();
    

    //private final DifferentialDrive drive;
    private final DifferentialDriveKinematics driveKinematics =
    new DifferentialDriveKinematics(Constants.DriveConstants.kTrackwidthMeters);

    // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // The robot's drive for odometry
  //private final DifferentialDrive mm_drive =
      //new DifferentialDrive(leftLeader::set, rightLeader::set);

    public CANDriveSubsystem() {

    // Sets the distance per pulse for the encoders
    // leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();


    m_odometry = new DifferentialDriveOdometry(
                    new Rotation2d(Units.degreesToRadians(gyro.getAngle())), 
            
                    -leftEncoder.getPosition() * Constants.DriveConstants.GEAR_RATIO * Constants.DriveConstants.WHEEL_CIRCUMFERENCE, 
            
                    -rightEncoder.getPosition() * Constants.DriveConstants.GEAR_RATIO * Constants.DriveConstants.WHEEL_CIRCUMFERENCE);


      /** Resets the drive encoders to currently read a position of 0. */
  




// -------------------------------OLD TEMPLATE CODE----------------------------------

        // set up differential drive class
        //drive = new DifferentialDrive(leftLeader, rightLeader);

    

        // Create the configuration to apply to motors. Voltage compensation
        // helps the robot perform more similarly on different
        // battery voltages (at the cost of a little bit of top speed on a fully charged
        // battery). The current limit helps prevent tripping
        // breakers.
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(12);
        config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
        config.encoder.positionConversionFactor(DriveConstants.ENCODER_CONVERSION_FACTOR);
        // Set configuration to follow leader and then apply it to corresponding
        // follower. Resetting in case a new controller is swapped
        // in and persisting in case of a controller reset due to breaker trip
            // config.follow(leftLeader);
            // leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            // config.follow(rightLeader);
            // rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Remove following, then apply config to right leader
        //config.disableFollowerMode();
        rightLeader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        // Set conifg to inverted and then apply to left leader. Set Left side inverted
        // so that postive values drive both sides forward
        config.inverted(true);
        leftLeader.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        // ------------------------- Pathplanner Configuration below, but still in constructor -------------

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file


    RobotConfig robiconfig = null;
    try{
      robiconfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    //RobotConfig robiconfig = new RobotConfig(60, 80, new ModuleConfig(.076, 3.000, 1, neo, 40, 1), .546);



    // Configure AutoBuilder last
    
     AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getWheelSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            robiconfig, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    }







    @Override
    public void periodic() {  
         // Update the odometry in the periodic block
         //mm_drive.feed();



    m_odometry.update(
        new Rotation2d(Units.degreesToRadians(gyro.getAngle())), 

        -leftEncoder.getPosition() * Constants.DriveConstants.GEAR_RATIO * Constants.DriveConstants.WHEEL_CIRCUMFERENCE*18, //random trying to make it work good enough, units or something is off

        -rightEncoder.getPosition() * Constants.DriveConstants.GEAR_RATIO * Constants.DriveConstants.WHEEL_CIRCUMFERENCE*18); //random trying to make it work good enough, units or something is off

        SmartDashboard.putNumber("Gyro Rotation", gyro.getAngle());
        SmartDashboard.putNumber("Left Leader Value", -leftLeader.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Leader Value", -rightLeader.getEncoder().getPosition());
         SmartDashboard.putData("Field", m_field);
        SmartDashboard.putNumber("xSpeed",getWheelSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("ySpeed",getWheelSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("TargetLeftMPS", wheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("TargetRightMPS", wheelSpeeds.rightMetersPerSecond);
         m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    
    public Command driveArcade(CANDriveSubsystem driveSubsystem, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
      return Commands.run(() -> {
          double leftSpeed = xSpeed.getAsDouble() + zRotation.getAsDouble();
          double rightSpeed = xSpeed.getAsDouble() - zRotation.getAsDouble();
  
          // Apply speeds to motors
          driveSubsystem.leftLeader.set(leftSpeed);
          driveSubsystem.rightLeader.set(rightSpeed);
      }, driveSubsystem); // This command requires the drive subsystem
  }
      
  

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
      }

      public Pose2d getPose() {
    return m_odometry.getPoseMeters(); }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
 
  public ChassisSpeeds getWheelSpeeds() {
    wheelSpeeds = new DifferentialDriveWheelSpeeds(
        leftEncoder.getVelocity()/1000,// * Constants.DriveConstants.RPMtoMPS
        rightEncoder.getVelocity()/1000 //* Constants.DriveConstants.RPMtoMPS
    );

    return driveKinematics.toChassisSpeeds(wheelSpeeds);
}
  

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        new Rotation2d(Units.degreesToRadians(gyro.getAngle())),
         -leftEncoder.getPosition() * Constants.DriveConstants.GEAR_RATIO * Constants.DriveConstants.WHEEL_CIRCUMFERENCE, 
         -rightEncoder.getPosition() * Constants.DriveConstants.GEAR_RATIO * Constants.DriveConstants.WHEEL_CIRCUMFERENCE, 
         pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
//   public void arcadeDrive(double fwd, double rot) {
//     mm_drive.arcadeDrive(fwd, rot);

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
//    */
//   public void tankDriveVolts(double leftVolts, double rightVolts) {
//     m_leftLeader.setVoltage(leftVolts);
//     m_rightLeader.setVoltage(rightVolts);
//     m_drive.feed();
 
public double getAverageEncoderDistance() {
    return ((-leftEncoder.getPosition() * Constants.DriveConstants.GEAR_RATIO * Constants.DriveConstants.WHEEL_CIRCUMFERENCE) 
    + (-rightEncoder.getPosition() * Constants.DriveConstants.GEAR_RATIO * Constants.DriveConstants.WHEEL_CIRCUMFERENCE)) / 2.0;
  }

 /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
 

  /** Zeroes the heading of the robot. */
  public void resetgyro() {
    gyro.reset();
  }


   /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return new Rotation2d(Units.degreesToRadians(gyro.getAngle())).getDegrees();
  }


  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }

  
            
  

  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert the desired chassis motion (vx, omega) into left/right wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(speeds);

    // Convert to motor commands
    double leftVelocitySetpoint = wheelSpeeds.leftMetersPerSecond;
    double rightVelocitySetpoint = wheelSpeeds.rightMetersPerSecond;
    

    // Use PID (and optionally feedforward) to calculate motor voltages
    double leftOutput = leftPID.calculate(
        -leftEncoder.getVelocity(),   // measured velocity (m/s)
        leftVelocitySetpoint         // target velocity (m/s)
    );
    double rightOutput = rightPID.calculate(
        -rightEncoder.getVelocity(),
        rightVelocitySetpoint
    );

    // Apply to motors
      leftLeader.setVoltage(leftPID.calculate(-leftEncoder.getVelocity(), leftVelocitySetpoint)-(leftVelocitySetpoint*1.4));
      rightLeader.setVoltage(rightPID.calculate(-rightEncoder.getVelocity(), rightVelocitySetpoint)-(rightVelocitySetpoint*1.4));

      // leftLeader.set(-leftVelocitySetpoint/15);
      // rightLeader.set(-rightVelocitySetpoint/15);
    
}
    
}
