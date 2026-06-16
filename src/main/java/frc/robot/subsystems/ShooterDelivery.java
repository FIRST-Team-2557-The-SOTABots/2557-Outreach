// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class ShooterDelivery extends SubsystemBase {
    private SparkFlexConfig shooterDeliveryConfig;
    private SparkFlexConfig shooterDelivery2Config;
    private SparkFlex shooterDelivery;
    private SparkFlex shooterDelivery2;

  /** Creates a new ShooterDelivery. */
  public ShooterDelivery() {

        shooterDelivery = new SparkFlex(
      Constants.ShooterConstants.kShooterDelivery1CanId, 
      MotorType.kBrushless);

          shooterDeliveryConfig = Configs.ShooterConfigs.ShooterDeliveryConfig;

              shooterDelivery.configure(
      shooterDeliveryConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);


      shooterDelivery2 = new SparkFlex(
      Constants.ShooterConstants.kShooterDelivery2CanId, 
      MotorType.kBrushless);

          shooterDelivery2Config = Configs.ShooterConfigs.ShooterDelivery2Config;

              shooterDelivery2.configure(
      shooterDelivery2Config,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);
      shooterDelivery2Config.inverted(true);
  }

    public void setDeliveryVoltage(double voltage) {
    shooterDelivery.setVoltage(voltage);
    shooterDelivery2.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
