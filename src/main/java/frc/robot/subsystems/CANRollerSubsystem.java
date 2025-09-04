// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {
  TalonSRX rollerMotor = new TalonSRX(5);

  public CANRollerSubsystem() {

  }

  @Override
  public void periodic() {
  }

  public Command runRoller(
      CANRollerSubsystem rollerSubsystem, double speed) {
    return Commands.run(
        () -> rollerMotor.set(TalonSRXControlMode.PercentOutput, speed), rollerSubsystem);
  }

  // Command to run the roller with joystick inputs. Comes with example kitbot code
  public Command runRollerExampleVariation(
      CANRollerSubsystem rollerSubsystem, DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.run(
        () -> rollerMotor.set(TalonSRXControlMode.PercentOutput,    forward.getAsDouble() - reverse.getAsDouble()), rollerSubsystem);
  }

}
