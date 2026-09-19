// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode; 
import com.revrobotics.spark.SparkBase.ResetMode;   
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private SparkMax intakeROT;
  private SparkAbsoluteEncoder intakeEncoder;
  private SparkClosedLoopController intakePID;

  // Use a local copy of the config so modifications do not affect other subsystems
  private SparkMaxConfig intakeRotConfig = new SparkMaxConfig();
  private double intakePositionTarget = Constants.IntakeConstants.IntakePosition.kStowed;

  // Tracks the shifted absolute encoder reading when fully UP against the hard stop
  private double intakeZeroOffset = 0.0;

  public Intake() {
    intakeROT = new SparkMax(
      Constants.IntakeConstants.kIntakeROTCanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    intakePID = intakeROT.getClosedLoopController();
    intakeEncoder = intakeROT.getAbsoluteEncoder();

    // Copy the base configuration from your global Configs class
    intakeRotConfig.apply(Configs.IntakeConfigs.intakeROTConfig);

    intakeRotConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(IntakeConstants.kIntakeROTkP, IntakeConstants.kIntakeROTkI, IntakeConstants.kIntakeROTkD);

    // Safe current limit so the motor can safely stall into the stop without melting
    intakeRotConfig.smartCurrentLimit(25);

    intakeROT.configure(
      intakeRotConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);

    // Establish the initial raw baseline position upon robot booting up
    intakeZeroOffset = intakeEncoder.getPosition();
  }

  /**
   * Gets the intake position adjusted for mid-match encoder slippage.
   * If it slips, this will still correctly output 0.0 when fully up against the hard stop.
   */
  public double getIntakePosition() {
    return intakeEncoder.getPosition() - intakeZeroOffset; 
  }

  public void setIntakePosition(double targetPosition) {
    this.intakePositionTarget = targetPosition;
  }

  public boolean isStowed() {
    return intakePositionTarget == Constants.IntakeConstants.IntakePosition.kStowed;
  }

  /**
   * Manually commands raw power override (bypassing PID) to drive into the hard stop during homing.
   */
  public void setRawPower(double percentOutput) {
    intakeROT.set(percentOutput);
  }

  @Override
  public void periodic() {
    double currentVelocity = intakeEncoder.getVelocity();
    double appliedOutput = intakeROT.getAppliedOutput();

    // --- AUTOMATIC STALL / SLIP DETECTION ---
    // If your code is commanding upward output and the velocity drops close to 0,
    // the intake has hit its physical top stop.
    if (appliedOutput > 0.1 && Math.abs(currentVelocity) < 0.05) {
      // Capture the broken, shifted raw encoder value as our new "Up/Stowed" baseline
      intakeZeroOffset = intakeEncoder.getPosition() - 0.01;
    }

    // Adjust your hardware target using the software offset before giving it to the SPARK MAX
    double correctedHardwareTarget = intakePositionTarget + intakeZeroOffset;

    // Arbitrary feed-forward to fight gravity
    double arbFF = -0.05; 

    // Using the correct, modern setSetpoint method
    intakePID.setSetpoint(
        correctedHardwareTarget, 
        ControlType.kPosition, 
        com.revrobotics.spark.ClosedLoopSlot.kSlot0, 
        arbFF
    );
    
    // Dashboard telemetry
    SmartDashboard.putNumber("Target Intake Pos", intakePositionTarget);
    SmartDashboard.putNumber("Corrected Hardware Target", correctedHardwareTarget);
    SmartDashboard.putNumber("Current Calibrated Position", getIntakePosition());
    SmartDashboard.putNumber("Raw Absolute Position", intakeEncoder.getPosition());
    SmartDashboard.putNumber("APPLIED OUTPUT", appliedOutput);
    SmartDashboard.putNumber("Intake Velocity", currentVelocity);
  }
}
