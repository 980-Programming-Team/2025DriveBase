// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants;

public class CANdleSystem extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.CANdle.kCANdleID, Constants.kCANivore);
  private CommandXboxController joystick;
  private int LedCount = Constants.LED_NUM;

  public CANdleSystem(CommandXboxController joy) {
    this.joystick = joy;
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.5;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.clearStickyFaults();
    m_candle.configAllSettings(configAll, 100);
  }

  public void SetLEDRed() {
    m_candle.setLEDs(140, 0, 0, 0, 0, LedCount);
  }

  public void SetLEDGreen() {
    m_candle.setLEDs(0, 140, 140, 0, 0, LedCount);
  }

  public void SetLEDYellow() {
    m_candle.setLEDs(140, 140, 0, 0, 0, LedCount);
  }

  public void SetLEDBlue() {
    m_candle.setLEDs(0, 0, 140, 0, 0, LedCount);
  }

  /* Wrappers so we can access the CANdle from the subsystem */
  public double getVbat() {
    return m_candle.getBusVoltage();
  }

  public double get5V() {
    return m_candle.get5VRailVoltage();
  }

  public double getCurrent() {
    return m_candle.getCurrent();
  }

  public double getTemperature() {
    return m_candle.getTemperature();
  }

  public void configBrightness(double percent) {
    m_candle.configBrightnessScalar(percent, 0);
  }

  public void configLos(boolean disableWhenLos) {
    m_candle.configLOSBehavior(disableWhenLos, 0);
  }

  public void configLedType(LEDStripType type) {
    m_candle.configLEDType(type, 0);
  }

  public void configStatusLedBehavior(boolean offWhenActive) {
    m_candle.configStatusLedState(offWhenActive, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
