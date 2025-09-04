// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANRollerSubsystem;


public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANRollerSubsystem rollerSubsystem = new CANRollerSubsystem();
  //private final CANRollerSubsystem rollerSubsystem = new CANRollerSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser;

  // --------- Lets you nerf the acceleration on the sticks -----------------------
  //private SlewRateLimiter transLimiter = new SlewRateLimiter(1.5);
  //private SlewRateLimiter rotlimiter = new SlewRateLimiter(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    //autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(driveSubsystem, rollerSubsystem));
  }


  private void configureBindings() {

// ---------------------------- Default Commands -----------------------------------

  rollerSubsystem.setDefaultCommand(
      rollerSubsystem.runRollerExampleVariation(rollerSubsystem, () -> 0.0, () -> 0.0));
      
  driveSubsystem.setDefaultCommand(
      driveSubsystem.driveArcade(
          driveSubsystem,
          () -> driverController.getLeftY(),
          () -> -driverController.getRightX()));

// ---------------------------- Driver P1 Controls ---------------------------------

    driverController.b().whileTrue(rollerSubsystem.runRollerExampleVariation(rollerSubsystem, () -> 0.5, () -> 0));
    driverController.a().whileTrue(rollerSubsystem.runRollerExampleVariation(rollerSubsystem, () -> 0, () -> 0.5));
    driverController.y().whileTrue(Commands.run(()-> driveSubsystem.driveRobotRelative(new ChassisSpeeds()), driveSubsystem));
  

// ---------------------------- Operator/P2 Controls -------------------------------

// Set the A button to run the "runRoller" command from the factory with a fixed value ejecting the gamepiece while the button is held from constants
    operatorController.a().whileTrue(rollerSubsystem.runRollerExampleVariation(rollerSubsystem, () -> RollerConstants.ROLLER_EJECT_VALUE, () -> 0));}


  
// ----------------------------- Automomous Mode Routine ---------------------------- 

  public Command getAutonomousCommand() {
    
    return autoChooser.getSelected();    //This should work, if not just uncomment and pick a command
    //return new PathPlannerAuto("W Tank Drive Auto");
    //return Autos.exampleAuto(driveSubsystem, rollerSubsystem);
  }
}
