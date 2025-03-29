// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class CANdleSystem extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.CANdle.kCANdleID, Constants.kCANivore);
  private int LedCount = Constants.LED_NUM;

  private boolean clearAnimations = false;
  private boolean m_animDirection = false;
  private boolean m_setAnim = false;
  private Animation m_toAnimate = null;
  private int candleIndex = 0;

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll,
    Empty
  }

  private AnimationTypes m_currentAnimation;

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

  public void toggleAnimDirection() {
    m_animDirection = !m_animDirection;
  }

  public void incrementAnimation() {

    switch (m_currentAnimation) {
      case ColorFlow:
        changeAnimation(AnimationTypes.Fire);
        break;
      case Fire:
        changeAnimation(AnimationTypes.Larson);
        break;
      case Larson:
        changeAnimation(AnimationTypes.Rainbow);
        break;
      case Rainbow:
        changeAnimation(AnimationTypes.RgbFade);
        break;
      case RgbFade:
        changeAnimation(AnimationTypes.SingleFade);
        break;
      case SingleFade:
        changeAnimation(AnimationTypes.Strobe);
        break;
      case Strobe:
        changeAnimation(AnimationTypes.Twinkle);
        break;
      case Twinkle:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
      case TwinkleOff:
        changeAnimation(AnimationTypes.Empty);
        break;
      case Empty:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
      case SetAll:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
    }
  }

  public void decrementAnimation() {
    switch (m_currentAnimation) {
      case ColorFlow:
        changeAnimation(AnimationTypes.Empty);
        break;
      case Fire:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
      case Larson:
        changeAnimation(AnimationTypes.Fire);
        break;
      case Rainbow:
        changeAnimation(AnimationTypes.Larson);
        break;
      case RgbFade:
        changeAnimation(AnimationTypes.Rainbow);
        break;
      case SingleFade:
        changeAnimation(AnimationTypes.RgbFade);
        break;
      case Strobe:
        changeAnimation(AnimationTypes.SingleFade);
        break;
      case Twinkle:
        changeAnimation(AnimationTypes.Strobe);
        break;
      case TwinkleOff:
        changeAnimation(AnimationTypes.Twinkle);
        break;
      case Empty:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
      case SetAll:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
    }
  }

  public void setColors() {
    changeAnimation(AnimationTypes.SetAll);
  }

  public void changeAnimation(AnimationTypes toChange) {
    m_currentAnimation = toChange;

    switch (toChange) {
      default:
      case ColorFlow:
        candleIndex = 0;
        m_toAnimate =
            new ColorFlowAnimation(
                128, 20, 70, 0, 0.7, LedCount - 8, Direction.Forward, candleIndex * LedCount);
        break;
      case Fire:
        candleIndex = 1;
        m_toAnimate =
            new FireAnimation(
                0.5, 0.7, LedCount - 8, 0.8, 0.5, m_animDirection, candleIndex * LedCount);
        break;
      case Larson:
        candleIndex = 2;
        m_toAnimate =
            new LarsonAnimation(
                0, 255, 46, 0, 0.1, LedCount - 8, BounceMode.Front, 3, candleIndex * LedCount);
        break;
      case Rainbow:
        candleIndex = 3;
        m_toAnimate =
            new RainbowAnimation(0.7, 0.7, LedCount - 8, m_animDirection, candleIndex * LedCount);
        break;
      case RgbFade:
        candleIndex = 4;
        m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount - 8, candleIndex * LedCount);
        break;
      case SingleFade:
        candleIndex = 5;
        m_toAnimate =
            new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount - 8, candleIndex * LedCount);
        break;
      case Strobe:
        candleIndex = 6;
        m_toAnimate =
            new StrobeAnimation(240, 10, 180, 0, 0.01, LedCount - 8, candleIndex * LedCount);
        break;
      case Twinkle:
        candleIndex = 7;
        m_toAnimate =
            new TwinkleAnimation(
                30, 70, 60, 0, 0.4, LedCount - 8, TwinklePercent.Percent42, candleIndex * LedCount);
        break;
      case TwinkleOff:
        candleIndex = 8;
        m_toAnimate =
            new TwinkleOffAnimation(
                70,
                90,
                175,
                0,
                0.2,
                LedCount - 8,
                TwinkleOffPercent.Percent76,
                candleIndex * LedCount);
        break;
      case Empty:
        candleIndex = 9;
        m_toAnimate =
            new RainbowAnimation(0.7, 0.7, LedCount - 8, m_animDirection, candleIndex * LedCount);
        break;

      case SetAll:
        m_toAnimate = null;
        break;
    }
    // System.out.println("Changed to " + m_currentAnimation.toString());
  }

  public void clearAllAnims() {
    clearAnimations = true;
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
