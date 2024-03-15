package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OIConstants;

public class LEDs extends SubsystemBase {
    private final AddressableLED m_leds;
    private final AddressableLEDBuffer m_ledBuffer;

    private Color curColor = OIConstants.kLedOrange;
    private Color activeColor = curColor;

    private double currentTime;
    private double lastTime;

    private boolean blinkEnabled = false;
    private boolean blinking = false;


    public LEDs(int pwmPort, int ledLength) {
        m_leds = new AddressableLED(pwmPort);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);

        m_leds.setLength(ledLength);
        m_leds.start();

        Thread flashThread = new Thread(() -> {
            while (true) {
                try {
                    Thread.sleep(300);
                    blinking = false;
                    Thread.sleep(300);
                    blinking = true;
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        flashThread.start();
    }

    @Override
    public void periodic() {
        m_leds.setData(m_ledBuffer);

        if (!blinkEnabled || (blinkEnabled && blinking)) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, curColor);
            }
        } else {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, Color.kBlack);
            }
        }

        // currentTime = Timer.getFPGATimestamp();
        // if (lastTime - currentTime >= 0.3) {
        //     lastTime = currentTime;
        //     cycleOn = true;
        // } else {
        //     cycleOn = false;
        // }

        SmartDashboard.putString("LedColor", curColor.toHexString());
    }

    public Command setColor(Color color) {
        return runOnce(
            () -> curColor = color
        );
    }

    public Command setBlink(boolean enabled) {
        return runOnce(
            () -> blinkEnabled = enabled
        );
    }

    public void enable() {
        m_leds.start();
    }

    public void disable() {
        m_leds.stop();
    }
}