package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final SparkMax intakeROT;
  private final SparkAbsoluteEncoder intakeEncoder;

  // Standard WPILib PIDController running on the roboRIO
  private final PIDController intakePID = new PIDController(
      IntakeConstants.kIntakeROTkP,
      IntakeConstants.kIntakeROTkI,
      IntakeConstants.kIntakeROTkD
  );

  private double intakePositionTarget = IntakeConstants.IntakePosition.kStowed;
  private double intakeZeroOffset = 0.0;
  private boolean isManualMode = false;

  public Intake() {
    intakeROT = new SparkMax(
        IntakeConstants.kIntakeROTCanId,
        com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    intakeEncoder = intakeROT.getAbsoluteEncoder();

    SparkMaxConfig intakeRotConfig = new SparkMaxConfig();
    intakeRotConfig.apply(Configs.IntakeConfigs.intakeROTConfig);
    intakeRotConfig.smartCurrentLimit(40);

    intakeROT.configure(
        intakeRotConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    intakeZeroOffset = intakeEncoder.getPosition();
  }

  public double getIntakePosition() {
    return intakeEncoder.getPosition() - intakeZeroOffset;
  }

  public boolean isStowed() {
    return this.intakePositionTarget == IntakeConstants.IntakePosition.kStowed;
  }

  public void setIntakePosition(double targetPosition) {
    this.isManualMode = false;
    this.intakePositionTarget = targetPosition;
  }

  public void setRawPower(double percentOutput) {
    this.isManualMode = true;
    intakeROT.set(percentOutput);
  }

  @Override
  public void periodic() {
    double currentVelocity = intakeEncoder.getVelocity();
    double appliedOutput = intakeROT.getAppliedOutput();

    // Automatic stall detection baseline calibration
    if (appliedOutput > 0.1 && Math.abs(currentVelocity) < 0.05) {
      intakeZeroOffset = intakeEncoder.getPosition() - 0.01;
    }

    double currentPos = intakeEncoder.getPosition();
    double correctedHardwareTarget = intakePositionTarget + intakeZeroOffset;

    if (!isManualMode) {
      // Calculate output using WPILib PID on the RIO
      double pidOutput = intakePID.calculate(currentPos, correctedHardwareTarget);
      double arbFF = -0.05;

      // Clamp total output between -1.0 and 1.0
      double totalOutput = Math.max(-1.0, Math.min(1.0, pidOutput + arbFF));
      intakeROT.set(totalOutput);
    }

    // Telemetry
    SmartDashboard.putNumber("Target Intake Pos", intakePositionTarget);
    SmartDashboard.putNumber("Corrected Hardware Target", correctedHardwareTarget);
    SmartDashboard.putNumber("Current Calibrated Position", getIntakePosition());
    SmartDashboard.putNumber("Raw Absolute Position", currentPos);
    SmartDashboard.putNumber("APPLIED OUTPUT", appliedOutput);
    SmartDashboard.putNumber("Is Manual Mode", isManualMode ? 1 : 0);
  }
}