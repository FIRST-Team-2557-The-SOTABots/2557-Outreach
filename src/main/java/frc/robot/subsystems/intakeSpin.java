// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class intakeSpin extends SubsystemBase {

        private SparkFlex intake;
        private SparkFlexConfig intakeConfig;
        private SparkFlex intake2;
        private SparkFlexConfig intakeConfig2;
  /** Creates a new intakeSpin. */
  public intakeSpin() {
    intake = new SparkFlex(
      Constants.IntakeConstants.kIntakeIntakeCanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

          intakeConfig = Configs.IntakeConfigs.intakeConfig;

              intake.configure(
      intakeConfig,
     ResetMode.kNoResetSafeParameters,
     PersistMode.kPersistParameters);


     intake2 = new SparkFlex(
      Constants.IntakeConstants.kIntakeIntake2CanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

          intakeConfig2 = Configs.IntakeConfigs.intakeConfig2;
              intake2.configure(
      intakeConfig2,
     ResetMode.kNoResetSafeParameters,
     PersistMode.kPersistParameters);
    intakeConfig2.inverted(true)
    .smartCurrentLimit(80);

  }

  
  public void setIntakeVoltage(double voltage) {
    intake.setVoltage(voltage);
    intake2.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
