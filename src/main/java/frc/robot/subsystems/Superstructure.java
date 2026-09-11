// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import choreo.auto.AutoFactory;
//import choreo.auto.AutoRoutine;
//import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GlobalConstants;
//import frc.robot.simulation.BallLaunchHelper;
//import frc.robot.simulation.GamePieceConstants;
//import frc.robot.simulation.GamePieceSimulation;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.floor.Floor;
import frc.robot.subsystems.floor.FloorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.Extension.ExtensionSetpoint;
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

  public Trigger isShooting = new Trigger(() -> shooting);

  private boolean intakeIn = true;

  //private final GamePieceSimulation fuelSim;

  public Superstructure(
      MAXSwerve drivebase, Floor floor, Intake intake, Flywheel flywheel, Pivot pivot) {
    this.drivebase = drivebase;
    this.floor = floor;
    this.intake = intake;
    this.flywheel = flywheel;
    this.pivot = pivot;

    //fuelSim = GamePieceSimulation.getInstance();

    isShooting.onTrue(RunIndexer());
    isShooting.onFalse(StopIndexer());

  }

  Command RunIndexer() {
      return Commands.parallel(
        Commands.repeatingSequence(
            FloorForward(), Commands.waitSeconds(3), FloorReverse(), Commands.waitSeconds(0.125)),
        Commands.either(
          Commands.repeatingSequence(
            intake.changeSetpoint(IntakeConstants.Roller.SHUFFLE),
            intake.changeSetpoint(ExtensionSetpoint.SHUFFLE_OUT),
            Commands.waitUntil(intake::atSetpoint),
            intake.changeSetpoint(ExtensionSetpoint.SHUFFLE_IN),
            Commands.waitUntil(intake::atSetpoint)), 
          Commands.none(),
          () -> intakeIn));
  }

  Command StopIndexer() {
    return Commands.parallel(FloorOff(), intake.changeSetpoint(0));
  }

    @param message 
    @return
   
  public Command logMessage(String message) {
    return Commands.runOnce(() -> Logger.recordOutput("Command Log", message));
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
