// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private SparkFlex shooterFlyWheelLeft;
  private SparkFlex shooterFlyWheelMiddle;
  private SparkFlex shooterFlyWheelRight;
  private SparkFlex shooterFlyWheelOther;

  private RelativeEncoder shooterFlyWheelLeftRelativeEncoder;
  private RelativeEncoder shooterFlyWheelMiddleRelativeEncoder;
  private RelativeEncoder shooterFlyWheelRightRelativeEncoder;
  private RelativeEncoder shooterFlyWheelOtherRelativeEncoder;

  private SparkFlexConfig shooterFlyWheelLeftConfig;
  private SparkFlexConfig shooterFlyWheelMiddleConfig;
  private SparkFlexConfig shooterFlyWheelRightConfig;
  private SparkFlexConfig shooterFlyWheelOtherConfig;

  private PIDController shooterFlyWheelPIDController;

  private DriveSubsystem kDrive;
  private Constants kConstants;
  private double targetRPM;
  private double holdVoltage;
  private double voltage;
  private boolean forward = true;

  private boolean manualShoot = true;
  // Initialize autoTarget to a safe starting distance (e.g., 2.0 meters)
  public double autoTarget = 2.0;

  /** Creates a new Shooter. */
  public Shooter(DriveSubsystem kDrive) {
    kConstants = new Constants();
    this.kDrive = kDrive;

    shooterFlyWheelLeft = new SparkFlex(
        Constants.ShooterConstants.kShooterFlyWheel1CanId,
        MotorType.kBrushless);
    shooterFlyWheelLeftRelativeEncoder = shooterFlyWheelLeft.getEncoder();

    shooterFlyWheelMiddle = new SparkFlex(
        Constants.ShooterConstants.kShooterFlyWheel2CanId,
        MotorType.kBrushless);
    shooterFlyWheelMiddleRelativeEncoder = shooterFlyWheelMiddle.getEncoder();

    shooterFlyWheelRight = new SparkFlex(
        Constants.ShooterConstants.kShooterFlyWheel3CanId,
        MotorType.kBrushless);
    shooterFlyWheelRightRelativeEncoder = shooterFlyWheelRight.getEncoder();

    shooterFlyWheelOther = new SparkFlex(
        Constants.ShooterConstants.kShooterFlyWheel4CanId,
        MotorType.kBrushless);
    shooterFlyWheelOtherRelativeEncoder = shooterFlyWheelOther.getEncoder();

    shooterFlyWheelLeftConfig = Configs.ShooterConfigs.ShooterFlyWheelLeftConfig;
    shooterFlyWheelMiddleConfig = Configs.ShooterConfigs.ShooterFlyWheelMiddleConfig;
    shooterFlyWheelRightConfig = Configs.ShooterConfigs.ShooterFlyWheelRightConfig;
    shooterFlyWheelOtherConfig = Configs.ShooterConfigs.ShooterFlyWheelOtherConfig;

    shooterFlyWheelLeft.configure(
        shooterFlyWheelLeftConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    shooterFlyWheelMiddle.configure(
        shooterFlyWheelMiddleConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    shooterFlyWheelRight.configure(
        shooterFlyWheelRightConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
    
    shooterFlyWheelOther.configure(
        shooterFlyWheelOtherConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    shooterFlyWheelPIDController = new PIDController(
        Constants.ShooterConstants.kFlyWheelP,
        Constants.ShooterConstants.kFlyWheelI,
        Constants.ShooterConstants.kFlyWheelD);

    shooterFlyWheelPIDController.setTolerance(Constants.ShooterConstants.kFlyWheelToleranceRPM);
  }

  public void incrementUp() {
    setAuto(); // Switch back to automatic mode when using POV
    autoTarget += 0.3;
    autoTarget = Math.min(autoTarget, 4.0); // Clamp to max map distance
  }

  public void incrementDown() {
    setAuto(); // Switch back to automatic mode when using POV
    autoTarget -= 0.3;
    autoTarget = Math.max(autoTarget, 1.2); // Clamp to min map distance
  }

  // Set target RPM directly (Automatic/Vision Mode)
  public void setFlyWheelRPM(double targetRPM, boolean forward) {
    this.targetRPM = targetRPM;
    this.forward = forward;
    setAuto();
  }

  // Set raw voltage directly (Manual Mode triggered by Right Trigger)
  public void setFlyWheelVoltage(double voltage) {
    this.voltage = voltage;
    setManual(); // Turns off PID and map hold
  }

  public double getHoldVoltage() {
    return Constants.ShooterConstants.kFlywheelMap.get(3.0);
  }

  public void shootAtTarget(Translation2d point, DriveSubsystem m_drive, double voltage) {
    if(m_drive.getPose() != null){
      double distance = m_drive.getPose().getTranslation().getDistance(point);
      double targetRPM = Constants.ShooterConstants.kFlywheelRPMMap.get(distance);
      setFlyWheelRPM(targetRPM, true);
    }
    else {
      setFlyWheelVoltage(voltage);
    }
    
  }  

  public boolean isAtSetpoint() {
    return shooterFlyWheelPIDController.atSetpoint();
  }

  public double getFlyWheelVelocity() {
    return (shooterFlyWheelLeftRelativeEncoder.getVelocity() +
            shooterFlyWheelMiddleRelativeEncoder.getVelocity() +
            shooterFlyWheelRightRelativeEncoder.getVelocity() +
            shooterFlyWheelOtherRelativeEncoder.getVelocity()) / 4.0;
  }

  public void setManual() {
    manualShoot = true;
  }

  public void setAuto() {
    manualShoot = false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("requested shooter RPM", targetRPM);
    SmartDashboard.putNumber("shooter RPM", getFlyWheelVelocity());
    SmartDashboard.putNumber("shooter voltage", voltage);
    SmartDashboard.putNumber("Shooter Hold Voltage", holdVoltage);
    SmartDashboard.putBoolean("forward", forward);
    SmartDashboard.putBoolean("shoot", isAtSetpoint());
    SmartDashboard.putBoolean("MANUAL SHOOTING", manualShoot);
    SmartDashboard.putNumber("AUTO TARGET (FEET)", autoTarget * 3.28084);
    SmartDashboard.putNumber("SHOOT DISTANCE FEET", (ShooterConstants.kDistRPMMap.get(getFlyWheelVelocity()) * 3.28084));
    SmartDashboard.putNumber("SHOOT DISTANCE METERS", (ShooterConstants.kDistRPMMap.get(getFlyWheelVelocity())));

    if (manualShoot) {
      // -------------------------------------------------------------
      // MANUAL MODE: Right trigger is held, apply raw voltage directly
      // -------------------------------------------------------------
      shooterFlyWheelLeft.setVoltage(voltage);
      shooterFlyWheelMiddle.setVoltage(voltage);
      shooterFlyWheelRight.setVoltage(voltage);
      shooterFlyWheelOther.setVoltage(voltage);
    } else {
      // -------------------------------------------------------------
      // AUTOMATIC MODE: Look up target RPM from autoTarget distance map
      // -------------------------------------------------------------
      targetRPM = Constants.ShooterConstants.kFlywheelRPMMap.get(autoTarget);

      if (targetRPM == 0) {
        shooterFlyWheelLeft.setVoltage(0);
        shooterFlyWheelMiddle.setVoltage(0);
        shooterFlyWheelRight.setVoltage(0);
        shooterFlyWheelOther.setVoltage(0);
      } else if (forward) {
        voltage = Math.max(-12.0, Math.min(
          12.0, shooterFlyWheelPIDController.calculate(
            getFlyWheelVelocity(), targetRPM)));

        holdVoltage = Constants.ShooterConstants.kFlywheelMap.get(targetRPM);

        double totalVoltage = voltage + holdVoltage;
        shooterFlyWheelLeft.setVoltage(totalVoltage);
        shooterFlyWheelMiddle.setVoltage(totalVoltage);
        shooterFlyWheelRight.setVoltage(totalVoltage);
        shooterFlyWheelOther.setVoltage(totalVoltage);
      } else {
        voltage = Math.max(-12.0, Math.min(
          12.0, shooterFlyWheelPIDController.calculate(
            getFlyWheelVelocity(), targetRPM)));

        holdVoltage = Constants.ShooterConstants.kFlywheelMap.get(targetRPM);

        double totalVoltage = (voltage + holdVoltage) * -1;
        shooterFlyWheelLeft.setVoltage(totalVoltage);
        shooterFlyWheelMiddle.setVoltage(totalVoltage);
        shooterFlyWheelRight.setVoltage(totalVoltage);
        shooterFlyWheelOther.setVoltage(totalVoltage);
      }
    }
  }
}