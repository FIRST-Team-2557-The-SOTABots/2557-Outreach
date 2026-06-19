// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode; 
import com.revrobotics.spark.SparkBase.ResetMode;   
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

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
  private double intakePosition = Constants.IntakeConstants.IntakePosition.kStowed;

  public Intake() {
    intakeROT = new SparkMax(
      Constants.IntakeConstants.kIntakeROTCanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    intakePID = intakeROT.getClosedLoopController();
    intakeEncoder = intakeROT.getAbsoluteEncoder();

    // Copy the base configuration from your global Configs class
    intakeRotConfig.apply(Configs.IntakeConfigs.intakeROTConfig);

    intakeROT.configure(
      intakeRotConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  public double getIntakePosition() {
    // Fixed: REVLib's getPosition() already returns a primitive double.
    return intakeEncoder.getPosition(); 
  }

  public void setIntakePosition(double position) {
    this.intakePosition = position;
  }

  /**
   * Recalibrates the absolute encoder zero-offset so the current physical 
   * resting position immediately streams as 0.01.
   */
  public void setUp(){
    // Fixed: Removed .getValue()
    double currentPhysicalPos = intakeEncoder.getPosition();

    // Compute the offset needed to force this spot to equal 0.01
    // Formula: Offset = Current Position - Target Position
    double newOffset = currentPhysicalPos - 0.01;

    // Update the configuration parameter
    intakeRotConfig.absoluteEncoder.zeroOffset(newOffset);

    // Send the updated configuration back to the SPARK MAX hardware
    intakeROT.configure(
      intakeRotConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters); // Use kNoPersist Parameters to avoid burning out flash memory
  }

  public boolean isStowed() {
    return intakePosition == Constants.IntakeConstants.IntakePosition.kStowed;
  }

  @Override
  public void periodic() {
    // Fixed: SparkClosedLoopController uses setReference, not setSetpoint
    intakePID.setReference(intakePosition, ControlType.kPosition);
  }
}