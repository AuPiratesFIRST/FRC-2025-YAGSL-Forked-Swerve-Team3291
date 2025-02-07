// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double RIGHT_SIDE_REDUCTION = 0.2;

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = 4.4196;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }


  public static class Elevator {
    public static final double deadband = 5;

    public static final double maxVelocity = 0.3;
    public static final double maxAcceleration = 0.3;
    public static final double ks = 0;
    public static final double kg = 0;
    public static final double kv = 0;
    public static final double ka = 0;

    public static class PID {
      public static final double kp = 0.1;
      public static final double ki = 0.0;
      public static final double kd = 0.0;
    }

    //ids
    public static final int encoderID = 0; //Changed due to change
    public static final int motorLeadID = 20;
    public static final int motorFollowerID = 22;
    public static final int topLimitSwitchID = 9;

    public static final double encoderOffset = 0;

    //floor pos
    public static final double groundFloor = 0;//doesn't like 0
    public static final double topFloor = 3;//needs to be changes

    public static final double maxVoltage = 8;
  }


  public static class Intake {
    public static final double angleDeadband = 5;

    public static final double kLauncherSubMinOutput = 0;
    public static final double kLauncherSubMaxOutput = 1;

    public static final int kLauncherSubLeftMotorId = 18;
    public static final int kLauncherSubRightMotorId = 14;

    public static final double kLauncherSubP = 0.00005;
    public static final double kLauncherSubI = 0.0;
    public static final double kLauncherSubD = 0.0;
    public static final double kLauncherSubFF = 0.0002;

    public static class intakePID {
      public static final double kp = 0.1;
      public static final double ki = 0.0;
      public static final double kd = 0.0;
      public static final double kcos = 0.5;
    }

    //ids
    public static final int encoderID = 0; //Changed due to change
    public static final int IntakeID = 21;//21
    public static final int PivotID = 19;//19 
    public static final int intakeLimitSwitchID = 9;

    public static final double k_pivotEncoderOffset = 230;

    //angles
    public static final double groundAngle = 167 - -35;//doesn't like 0
    public static final double stowAngle = 167 - 164;
    public static final double sourceAngle = 167 - 55;
    public static final double ampAngle = 167 - 95;

    public static final double maxPivotVoltage = 5;

    public static final double ejectSpeed = 0.5 * 5000;
    public static final double intakeSpeed = 0.7 * 5000;
    public static final int launchNoteTimeInSecs = 1;
    
  }
  
}
