package frc.robot.subsystems.swervedrive;

import swervelib.parser.PIDFConfig;
import frc.robot.subsystems.swervedrive.SwerveControllerConfiguration;
import frc.robot.subsystems.swervedrive.SwerveDriveConfiguration;

/** {@link swervelib.SwerveController} parsed class. Used to access the JSON data. */
public class ControllerPropertiesJson {

  /**
   * The minimum radius of the angle control joystick to allow for heading adjustment of the robot.
   */
  public double angleJoystickRadiusDeadband;
  /** The PID used to control the robot heading. */
  public PIDFConfig heading;

  /**
   * Create the {@link SwerveControllerConfiguration} based on parsed and given data.
   *
   * @param driveConfiguration {@link SwerveDriveConfiguration} parsed configuration.
   * @param maxSpeedMPS Maximum speed in meters per second for the angular acceleration of the
   *     robot.
   * @return {@link SwerveControllerConfiguration} object based on parsed data.
   */
  public SwerveControllerConfiguration createControllerConfiguration(
      SwerveDriveConfiguration driveConfiguration, double maxSpeedMPS) {
    return new SwerveControllerConfiguration(
        driveConfiguration, heading, angleJoystickRadiusDeadband, maxSpeedMPS);
  }
}
