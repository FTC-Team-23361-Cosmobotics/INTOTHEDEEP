import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lights {
    private final ColorSensorV3 colorSensor;
    private final RevBlinkinLedDriver blinkinLedDriver;

    public Lights(HardwareMap hardwareMap) {
        // Initialize color sensor and LED driver
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "LED");
    }

    public void updateLights() {
        // Get detected color
        Color detectedColor = colorSensor.getColor();
        
        // Log color values to SmartDashboard for debugging
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);

        // Determine LED pattern based on color
        if (detectedColor.red > 0.5 && detectedColor.green < 0.3 && detectedColor.blue < 0.3) {
            // Detected red
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (detectedColor.blue > 0.5) {
            // Detected blue
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (detectedColor.red > 0.5 && detectedColor.green > 0.5) {
            // Detected yellow (combination of red and green)
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        } else {
            // Default pattern
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
        }
    }
}
// Simply call updateLights() where needed
