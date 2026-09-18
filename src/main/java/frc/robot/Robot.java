package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.DrivebaseConstants.DriveConstants;
import frc.robot.subsystems.drive.DrivebaseConstants.MAXSwerveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIO_Real;
import frc.robot.subsystems.drive.MAXSwerve;
import frc.robot.subsystems.drive.MAXSwerveIO;
import frc.robot.subsystems.drive.MAXSwerveIO_Real;
import frc.robot.subsystems.drive.MAXSwerveIO_Relative;
import frc.robot.subsystems.drive.MAXSwerveIO_Sim;
import frc.robot.subsystems.floor.Floor;
import frc.robot.subsystems.floor.FloorIO_Real;
import frc.robot.subsystems.floor.FloorIO_Sim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO_Real;
// import frc.robot.subsystems.intake.IntakeIO_Sim;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO_Real;
// import frc.robot.subsystems.shooter.flywheel.FlywheelIO_Sim;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.subsystems.shooter.pivot.PivotIO_Real;
// import frc.robot.subsystems.shooter.pivot.PivotIO_Sim;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private final MAXSwerve drivebase;
  private final Floor floor;
  private final Intake intake;
  private final Flywheel flywheel;
  private final Pivot pivot;
  private final Superstructure superstructure;

  public static enum RobotMode {
    SIM,
    REPLAY,
    REAL
  }

  // Control the mode of the robot
  public static final RobotMode mode = Robot.isReal() ? RobotMode.REAL : RobotMode.SIM;

  // Auto Command
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");

  // Driver Controllers
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  public Robot() {

    drivebase =
        new MAXSwerve(
            mode == RobotMode.REAL ? new GyroIO_Real() : new GyroIO() {},
            mode == RobotMode.REAL
                ? new MAXSwerveIO[] {
                  new MAXSwerveIO_Real(DriveConstants.kFrontLeftSwerveModule),
                  new MAXSwerveIO_Real(DriveConstants.kFrontRightSwerveModule),
                  new MAXSwerveIO_Real(DriveConstants.kBackLeftSwerveModule),
                  new MAXSwerveIO_Relative(DriveConstants.kBackRightSwerveModule)
                }
                : new MAXSwerveIO[] {
                  new MAXSwerveIO_Sim(),
                  new MAXSwerveIO_Sim(),
                  new MAXSwerveIO_Sim(),
                  new MAXSwerveIO_Sim()
                });
    floor = new Floor(RobotBase.isReal() ? new FloorIO_Real() : new FloorIO_Sim());
    intake = new Intake(RobotBase.isReal() ? new IntakeIO_Real() : new IntakeIO_Real()); // TODO: Sim
    flywheel = new Flywheel(RobotBase.isReal() ? new FlywheelIO_Real() : new FlywheelIO_Real()); // TODO: Sim
    pivot = new Pivot(RobotBase.isReal() ? new PivotIO_Real() : new PivotIO_Real()); // TODO: Sim

    superstructure = new Superstructure(drivebase, floor, intake, flywheel, pivot);
      
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
  }

  @SuppressWarnings(value = "resource")
  @Override
  public void robotInit() {
    Logger.recordMetadata("Codebase", "6657 MAXSwerve 2026");
    switch (mode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
    }
    Logger.start();

    // Set the default command for the drivebase for TeleOP driving
    drivebase.setDefaultCommand(
        drivebase.runVelocityFieldRelative(
            () ->
                new ChassisSpeeds(
                    -MathUtil.applyDeadband(driver.getLeftY(), 0.05)
                        * MAXSwerveConstants.kMaxDriveSpeed
                        * 0.75,
                    -MathUtil.applyDeadband(driver.getLeftX(), 0.15)
                        * MAXSwerveConstants.kMaxDriveSpeed
                        * 0.75,
                    -MathUtil.applyDeadband(driver.getRightX(), 0.15)
                        * DriveConstants.kMaxAngularVelocity
                        * 0.5)));
    autoChooser.addOption("Nothing", Commands.print("Nothing Auto Selected"));
    autoChooser.addOption("Taxi", Commands.sequence(drivebase.runVelocity(() -> new ChassisSpeeds(1, 0, 0)).withTimeout(2)));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (autoChooser.get() != null) {
      CommandScheduler.getInstance().schedule(autoChooser.get());
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autoChooser.get() != null) {
      CommandScheduler.getInstance().cancel(autoChooser.get());
      CommandScheduler.getInstance().schedule(drivebase.stop());
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}
}
