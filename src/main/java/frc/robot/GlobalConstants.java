package frc.robot;

public class GlobalConstants {

  public static final double mainLoopFrequency = 50; // Hz

  public static enum CAN {
    Swerve_FL_D(1),
    Swerve_FR_D(2),
    Swerve_BL_D(3),
    Swerve_BR_D(4),
    Swerve_FL_T(5),
    Swerve_BL_T(7),
    Swerve_FR_T(6),
    Swerve_BR_T(8),
    Swerve_Pigeon(9),
    Intake_Pivot(10),
    Intake_Wheels(11),
    Floor(12),
    Ceiling_Wall(13),
    Shooter_Pivot(14),
    Shooter_Flywheel(15);

    public int id;

    CAN(int id) {
      this.id = id;
    }
  }
}
