// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.floor.Floor;
import frc.robot.subsystems.floor.FloorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.Roller;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.pivot.Pivot;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** The core class where all robot commands live. */
public class Superstructure {
  MAXSwerve drivebase;
  Floor floor;
  Intake intake;
  Flywheel flywheel;
  Pivot pivot;

  @AutoLogOutput(key = "RobotStates/Shooting")
  public boolean shooting = false;

  @AutoLogOutput(key = "RobotStates/KeepRoller")
  public boolean keepRoller = false;

  public Trigger isShooting = new Trigger(() -> shooting);

  public Superstructure(
      MAXSwerve drivebase, Floor floor, Intake intake, Flywheel flywheel, Pivot pivot) {
    this.drivebase = drivebase;
    this.floor = floor;
    this.intake = intake;
    this.flywheel = flywheel;
    this.pivot = pivot;

    isShooting.onTrue(RunIndexer());
    isShooting.onFalse(StopIndexer());
  }

  Command RunIndexer() {
    return Commands.parallel(
        Commands.sequence(
            intake.changeSetpointP(120), pivot.changeSetpointC(10), flywheel.changeSetpointC(3000)),
        Commands.repeatingSequence(
            FloorForward(), Commands.waitSeconds(3), FloorReverse(), Commands.waitSeconds(0.125)));
  }

  Command StopIndexer() {
    return Commands.parallel(FloorOff(), intake.changeSetpointP(0), flywheel.changeSetpointC(0));
  }

  // #region State Toggles
  public Command ToggleShooting() {
    return Commands.runOnce(() -> shooting = !shooting);
  }

  public Command EnableShooting() {
    return Commands.runOnce(() -> shooting = true);
  }

  public Command DisableShooting() {
    return Commands.runOnce(() -> shooting = false);
  }

  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
  }

  // #region Helper Sequences

  public Command HomeRobot() {
    return Commands.sequence(
        logMessage("Home Robot"),
        flywheel.changeSetpointC(0),
        floor.changeSetpoint(FloorConstants.Off),
        DisableShooting(),
        intake.changeSetpointP(30),
        intake.changeSetpointR(Roller.Off),
        pivot.changeSetpointC(0));
  }

  public Command ExtendIntake() {
    return Commands.sequence(
        logMessage("Fuel Intake"),
        intake.changeSetpointP(0),
        intake.changeSetpointR(Roller.FORWARD));
  }

  public Command RetractIntake() {
    if (keepRoller) {
      return Commands.sequence(
          logMessage("Intake Retracted, Roller Enabled"), intake.changeSetpointP(120));
    } else {
      return Commands.sequence(
          logMessage("Intake Retracted, Roller Disabled"),
          intake.changeSetpointP(120),
          intake.changeSetpointR(0));
    }
  }

  public Command ToggleRoller() {
    return Commands.runOnce(() -> keepRoller = !keepRoller);
  }

  public Command EnableDump() {
    return Commands.sequence(
        intake.changeSetpointR(Roller.FORWARD),
        intake.changeSetpointP(0),
        flywheel.changeSetpointC(-1000),
        FloorReverse());
  }

  public Command DisableDump() {
    return Commands.sequence(
        intake.changeSetpointR(Roller.Off), flywheel.changeSetpointC(0), FloorOff());
  }

  public Command FloorForward() {
    return Commands.sequence(
        logMessage("Floor Forward"), floor.changeSetpoint(FloorConstants.FORWARD));
  }

  public Command FloorReverse() {
    return Commands.sequence(
        logMessage("Floor Reverse"), floor.changeSetpoint(FloorConstants.REVERSE));
  }

  public Command FloorOff() {
    return Commands.sequence(logMessage("Floor Off"), floor.changeSetpoint(FloorConstants.Off));
  }
}
