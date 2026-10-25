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

  // Standard linear PIDController operating in our safe virtual space (0.0 to 0.75)
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

  /**
   * Converts raw absolute encoder position [0.0, 1.0) into a clean 
   * virtual linear position [0.0, 0.75], completely cutting out the 
   * [0.7, 0.95] dead zone.
   */
  private double rawToVirtual(double rawPos) {
    // Apply offset and normalize to [0, 1)
    double normalized = (rawPos - intakeZeroOffset) % 1.0;
    if (normalized < 0) {
      normalized += 1.0;
    }

    // Map the valid range around the dead zone
    if (normalized >= 0.95) {
      return normalized - 0.95; // Maps 0.95..1.0 to 0.0..0.05
    } else if (normalized <= 0.7) {
      return normalized + 0.05; // Maps 0.0..0.7 to 0.05..0.75
    } else {
      // If caught inside the dead zone, clamp to the nearest valid edge
      return (normalized - 0.7 < 0.95 - normalized) ? 0.75 : 0.0;
    }
  }

  /**
   * Converts virtual linear position back to raw absolute encoder units 
   * so the motor controller knows where to go.
   */
  private double virtualToRaw(double virtualPos) {
    double rawNormalized;
    if (virtualPos <= 0.05) {
      rawNormalized = virtualPos + 0.95;
    } else {
      rawNormalized = virtualPos - 0.05;
    }
    return (rawNormalized + intakeZeroOffset) % 1.0;
  }

  public double getIntakePosition() {
    return rawToVirtual(intakeEncoder.getPosition());
  }

  public boolean isStowed() {
    return this.intakePositionTarget == IntakeConstants.IntakePosition.kStowed;
  }

  public void setIntakePosition(double targetPosition) {
    this.isManualMode = false;
    // Clamp target to ensure it stays within the safe virtual range [0.0, 0.75]
    this.intakePositionTarget = Math.max(0.0, Math.min(0.75, targetPosition));
  }

  public void setRawPower(double percentOutput) {
    this.isManualMode = true;
    intakeROT.set(percentOutput);
  }

  @Override
  public void periodic() {
    double currentVelocity = intakeEncoder.getVelocity();
    double appliedOutput = intakeROT.getAppliedOutput();
    double currentVirtualPos = getIntakePosition();

    if (!isManualMode) {
      // Calculate output using standard PID on our dead-zone-free virtual position
      double pidOutput = intakePID.calculate(currentVirtualPos, intakePositionTarget);
      double arbFF = -0.05;

      // Clamp total output between -1.0 and 1.0
      double totalOutput = Math.max(-1.0, Math.min(1.0, pidOutput + arbFF));
      
      // Convert target back to raw space to command the motor safely
      double rawTarget = virtualToRaw(intakePositionTarget);
      
      // Safety check: ensure raw output position never commands into the dead zone
      double currentRaw = (intakeEncoder.getPosition() - intakeZeroOffset) % 1.0;
      if (currentRaw < 0) currentRaw += 1.0;
      
      if (currentRaw > 0.7 && currentRaw < 0.95) {
        intakeROT.set(0.0); // Stop if somehow trapped
      } else {
        intakeROT.set(totalOutput);
      }
    }

    // Telemetry
    SmartDashboard.putNumber("Target Virtual Pos", intakePositionTarget);
    SmartDashboard.putNumber("Current Virtual Pos", currentVirtualPos);
    SmartDashboard.putNumber("Raw Absolute Position", intakeEncoder.getPosition());
    SmartDashboard.putNumber("APPLIED OUTPUT", appliedOutput);
    SmartDashboard.putNumber("Is Manual Mode", isManualMode ? 1 : 0);
  }
}