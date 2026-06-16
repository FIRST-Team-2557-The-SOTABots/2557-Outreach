// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Delivery extends SubsystemBase {

  public static SparkFlex delivery;
  public int stutterCounter;

  public static SparkFlexConfig deliveryConfig;
  /** Creates a new Delivery. */
  public Delivery() {
    delivery = new SparkFlex(
      Constants.DeliveryConstants.kDeliveryCanId, 
      MotorType.kBrushless);

      
    deliveryConfig = Configs.DeliveryConfigs.deliveryConfig;
    

    delivery.configure(
      deliveryConfig,
      com.revrobotics.ResetMode.kNoResetSafeParameters,
      com.revrobotics.PersistMode.kPersistParameters);
  }

  public void setDeliveryVoltage(double voltage) {
    delivery.setVoltage(voltage);
  }

  public void resetStutterStop(){
    stutterCounter = 0;
    delivery.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
