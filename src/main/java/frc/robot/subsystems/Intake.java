// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
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


  private SparkMaxConfig intakeRotConfig;

  private double intakePosition = Constants.IntakeConstants.IntakePosition.kStowed;

  public Intake() {
    

    intakeROT = new SparkMax(
      Constants.IntakeConstants.kIntakeROTCanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    

    intakePID = intakeROT.getClosedLoopController();

    intakeEncoder = intakeROT.getAbsoluteEncoder();



    intakeRotConfig = Configs.IntakeConfigs.intakeROTConfig;




    intakeROT.configure(
      intakeRotConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters);
  }


  public double getIntakePosition() {
    return intakeEncoder.getPosition();
  }

  public void setIntakePosition(double position) {
    this.intakePosition = position;
  }

  public boolean isStowed() {
  return intakePosition == Constants.IntakeConstants.IntakePosition.kStowed;
}

  @Override
  public void periodic() {
    intakePID.setSetpoint(intakePosition, ControlType.kPosition);
  }
}
