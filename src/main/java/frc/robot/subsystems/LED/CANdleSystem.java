// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class CANdleSystem extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.CANdle.kCANdleID, Constants.kCANivore);
  private int LedCount = Constants.LED_NUM;

  public CANdleSystem() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.5;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.clearStickyFaults();
    m_candle.configAllSettings(configAll, 100);
  }

  public void SetLEDColor(int red, int green, int blue) {
    m_candle.setLEDs(red, green, blue, 0, 0, LedCount);
  }

  public void SetLEDRed() {
    SetLEDColor(140, 0, 0);
  }

  public void SetLEDGreen() {
    SetLEDColor(0, 140, 0);
  }

  public void SetLEDYellow() {
    SetLEDColor(140, 140, 0);
  }

  public void SetLEDBlue() {
    SetLEDColor(0, 0, 140);
  }

  public void SetLEDPurple0() {
    SetLEDColor(190, 90, 190);
  }

  public void SetLEDPurple1() {
    SetLEDColor(100, 0, 70);
  }

  public void SetLEDPurple2() {
    SetLEDColor(120, 10, 80);
  }

  public void SetLEDPurple3() {
    SetLEDColor(130, 20, 100);
  }

  public void SetLEDPurple4() {
    SetLEDColor(140, 0, 140);
  }

  public void SetLEDOrange() {
    SetLEDColor(180, 110, 0);
  }

  public void SetLEDOff() {
    m_candle.setLEDs(0, 0, 0, 0, 0, LedCount);
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
    // Ensure that this method is efficient and does not contain unnecessary operations
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
