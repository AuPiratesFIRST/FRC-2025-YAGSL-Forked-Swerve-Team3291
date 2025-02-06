package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.intake;

public class IntakeMotorSubsystem extends SubsystemBase {

  /*-------------------------------- public instance variables ---------------------------------*/
  public SparkMax IntakeMotorMotor;

  public SparkClosedLoopController IntakeMotorPID;

  public RelativeEncoder IntakeMotorEncoder;

  public SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(1000);
  public double intakeSpeed = Constants.intake.ejectSpeed;
  public SparkMaxConfig config;

  public IntakeMotorSubsystem() {
    Preferences.initDouble("intakeSpeed", intakeSpeed);

    IntakeMotorMotor = new SparkMax(Constants.intake.IntakeID, MotorType.kBrushless);
   // IntakeMotorMotor.restoreFactoryDefaults();

    IntakeMotorPID = IntakeMotorMotor.getClosedLoopController();
    config = new SparkMaxConfig();
    config
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    config.closedLoop.p(Constants.intake.kLauncherSubP);
    config.closedLoop.i(Constants.intake.kLauncherSubI);
    config.closedLoop.d(Constants.intake.kLauncherSubD);
    config.closedLoop.pidf(Constants.intake.kLauncherSubP, Constants.intake.kLauncherSubI, 
    Constants.intake.kLauncherSubD, Constants.intake.kLauncherSubFF);
    config.closedLoop.outputRange(Constants.intake.kLauncherSubMinOutput, Constants.intake.kLauncherSubMaxOutput);
    IntakeMotorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakeMotorEncoder = IntakeMotorMotor.getEncoder();

  //  IntakeMotorMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);


  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/
  public void moveIntakeMotor(double rpm) {
    IntakeMotorMotor.setInverted(true);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    // this.config.
    this.IntakeMotorPID.setReference(limitedSpeed, ControlType.kVelocity);
  }
  public void moveIntakeMotorReversed(double rpm) {
    IntakeMotorMotor.setInverted(false);
    double limitedSpeed = mSpeedLimiter.calculate(rpm);
    IntakeMotorPID.setReference(limitedSpeed, ControlType.kVelocity);
  }

  public void stopIntakeMotorSubsystem() {
    //double limitedSpeed = mSpeedLimiter.calculate(0);
    IntakeMotorPID.setReference(0, ControlType.kVelocity);
  }

  /*---------------------------------- Custom public Functions ---------------------------------*/
}


