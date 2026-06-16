package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final TalonFXConfiguration drivingConfig = new TalonFXConfiguration();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;


                // Configure PID values for velocity control
                drivingConfig.Slot0.kP = Constants.DriveConstants.kDrivingP; // Adjust these values for your robot
                drivingConfig.Slot0.kI = Constants.DriveConstants.kDrivingI;
                drivingConfig.Slot0.kD = Constants.DriveConstants.kDrivingD;
                drivingConfig.Slot0.kV = Constants.DriveConstants.kDrivingkV; // Feedforward gain
    
                // Configure motor output
                drivingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                drivingConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
                drivingConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0) // radians per second
                    // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class ShooterConfigs{
        public static final SparkFlexConfig ShooterFlyWheelLeftConfig = new SparkFlexConfig();
        public static final SparkFlexConfig ShooterFlyWheelMiddleConfig = new SparkFlexConfig();
        public static final SparkFlexConfig ShooterFlyWheelRightConfig = new SparkFlexConfig();
        public static final SparkFlexConfig ShooterFlyWheelOtherConfig = new SparkFlexConfig();
        public static final SparkFlexConfig ShooterDeliveryConfig = new SparkFlexConfig();
        public static final SparkFlexConfig ShooterDelivery2Config = new SparkFlexConfig();

        static {
            ShooterFlyWheelLeftConfig
                .idleMode(IdleMode.kCoast)
                .inverted(true)
                .smartCurrentLimit(40);

            ShooterFlyWheelMiddleConfig
                .idleMode(IdleMode.kCoast)
                .inverted(true)
                .smartCurrentLimit(40);

            ShooterFlyWheelRightConfig
                .idleMode(IdleMode.kCoast)
                .inverted(false)
                .smartCurrentLimit(40);

            ShooterFlyWheelOtherConfig
                .idleMode(IdleMode.kCoast)
                .inverted(false)
                .smartCurrentLimit(40);

            ShooterDeliveryConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(40); 
            }
    }

    public static final class IntakeConfigs {
        public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
        public static final SparkFlexConfig intakeConfig2 = new SparkFlexConfig();
        public static final SparkMaxConfig intakeROTConfig = new SparkMaxConfig();

        static {
            intakeConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(80)
                    .inverted(true);

            intakeROTConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40)
                    .inverted(true);

            intakeROTConfig.absoluteEncoder
                    .inverted(true);

            intakeROTConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(
                        Constants.IntakeConstants.kIntakeROTkP,
                        Constants.IntakeConstants.kIntakeROTkI,
                        Constants.IntakeConstants.kIntakeROTkD)
                    .outputRange(-1, 1);
                
        }
    }

    public static final class DeliveryConfigs {
        public static final SparkFlexConfig deliveryConfig = new SparkFlexConfig();

        static {
            deliveryConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40)
                    .inverted(true);
        }
    }
              
    public static final class ClimberConfigs {
        public static final SparkFlexConfig leftClimberConfig = new SparkFlexConfig();
        public static final SparkFlexConfig rightClimberConfig = new SparkFlexConfig();

        static {
            leftClimberConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40)
                    .inverted(true);
            rightClimberConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40)
                    .inverted(false);

        }
    }
}
