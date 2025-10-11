package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX climberMotor;

  private VoltageOut climberVoltageRequest;
  private PIDController pidController =
      new PIDController(ClimberConstants.KP, ClimberConstants.KI, ClimberConstants.KD);

  private StatusSignal<Current> climberStatorCurrentStatusSignal;
  private StatusSignal<Current> climberSupplyCurrentStatusSignal;
  private StatusSignal<Voltage> climberVoltageStatusSignal;
  private StatusSignal<AngularVelocity> climberVelocityStatusSignal;
  private StatusSignal<Temperature> climberTemperatureStatusSignal;

  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(1)
          .withKI(0)
          .withKD(0)
          .withKS(0)
          .withKV(0)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  public ClimberIOTalonFX() {
    climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID);

    climberStatorCurrentStatusSignal = climberMotor.getStatorCurrent();
    climberSupplyCurrentStatusSignal = climberMotor.getSupplyCurrent();
    climberVoltageStatusSignal = climberMotor.getMotorVoltage();
    climberVelocityStatusSignal = climberMotor.getVelocity();
    climberTemperatureStatusSignal = climberMotor.getDeviceTemp();

    configureClimberMotor(climberMotor);

    climberMotor.setPosition(0.0);
    climberVoltageRequest = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        climberStatorCurrentStatusSignal,
        climberSupplyCurrentStatusSignal,
        climberVoltageStatusSignal,
        climberVelocityStatusSignal,
        climberTemperatureStatusSignal);

    inputs.climberMotorStatorCurrentAmps = climberStatorCurrentStatusSignal.getValueAsDouble();
    inputs.climberMotorSupplyCurrentAmps = climberSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.climberMotorVoltage = climberVoltageStatusSignal.getValueAsDouble();
    inputs.climberMotorPositionRotations = climberVelocityStatusSignal.getValueAsDouble();
    inputs.climberMotorTemperatureCelsius = climberTemperatureStatusSignal.getValueAsDouble();
    inputs.climberMotorPos = climberMotor.getPosition().getValueAsDouble();

    TalonFXConfiguration config = new TalonFXConfiguration();
    MotionMagicConfigs motionMagicConfigs = config.MotionMagic;

    this.climberMotor.getConfigurator().refresh(config);
  }

  @Override
  public void setClimberVoltage(double voltage) {
    climberMotor.setControl(climberVoltageRequest.withOutput(voltage));
  }

  @Override
  public void zeroPosition() {
    climberMotor.setPosition(0.0);
  }

  @Override
  public void setPosition(double position) {
    climberMotor.setPosition(position);
  }

  private void configureClimberMotor(TalonFX motor) {
    TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs climberMotorCurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(30))
            .withStatorCurrentLimitEnable(true);

    climberMotorCurrentLimits.SupplyCurrentLimit =
        ClimberConstants.CLIMBER_CONTINUOUS_CURRENT_LIMIT;
    climberMotorCurrentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_PEAK_CURRENT_LIMIT;
    climberMotorCurrentLimits.SupplyCurrentLowerTime =
        ClimberConstants.CLIMBER_PEAK_CURRENT_DURATION;
    climberMotorCurrentLimits.SupplyCurrentLimitEnable = true;
    climberMotorConfig.CurrentLimits = climberMotorCurrentLimits;

    climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climberMotorConfig.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

    climberMotorConfig.MotorOutput.Inverted =
        ClimberConstants.CLIMBER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(climberMotorConfig);
      if (status.isOK()) break;
    }
  }
}
