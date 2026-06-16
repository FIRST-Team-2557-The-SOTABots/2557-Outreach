// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;
import frc.robot.Constants;

public class MAXSwerveModule {
  private final TalonFX m_drivingTalonFX;
  private final SparkMax m_turningSpark;

  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_turningClosedLoopController;
  private VelocityVoltage m_drivingVelocityRequest;

  private SwerveModuleState correctedDesiredState;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_drivingTalonFX = new TalonFX(drivingCANId);
    
    // Apply configuration
    m_drivingTalonFX.getConfigurator().apply(Configs.MAXSwerveModule.drivingConfig);

        // Initialize velocity control request
    m_drivingVelocityRequest = new VelocityVoltage(0).withSlot(0);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingTalonFX.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(getDriveWheelSpeedMPS(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        getDriveWheelPositionMeters(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingTalonFX.setControl(m_drivingVelocityRequest.withVelocity(KrakenConvertion(correctedDesiredState.speedMetersPerSecond)));
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }


  public double KrakenConvertion(double speedMetersPerSecond) {
    double wheelCircumference = Constants.ModuleConstants.kWheelCircumferenceMeters;
    double driveReduction = Constants.ModuleConstants.kDrivingMotorReduction;
    double motorRPM = ((speedMetersPerSecond * 60)/wheelCircumference) * driveReduction;
    return motorRPM;
  }

  public double getRequestedMPS(){
    if (correctedDesiredState != null){
      return correctedDesiredState.speedMetersPerSecond;
    }else{
      return 0.0;
    }
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalonFX.setPosition(0);
  }

  public double getDriveWheelSpeedMPS(){
    double wheelCircumference = Constants.ModuleConstants.kWheelCircumferenceMeters;
    double driveReduction = Constants.ModuleConstants.kDrivingMotorReduction;
    return (((getDriveVelocityRPM() / driveReduction) * wheelCircumference)/60);
  }

  public double getDriveWheelPositionMeters(){
    double wheelCircumference = Constants.ModuleConstants.kWheelCircumferenceMeters;
    double driveReduction = Constants.ModuleConstants.kDrivingMotorReduction;
    return ((getDrivePosition() / driveReduction) * wheelCircumference);
  }

  public double getDriveVelocityRPM() {
    return m_drivingTalonFX.getVelocity().getValueAsDouble()*60.0;
  }

  public double getDrivePosition() {
    return m_drivingTalonFX.getPosition().getValueAsDouble();
  }
}
