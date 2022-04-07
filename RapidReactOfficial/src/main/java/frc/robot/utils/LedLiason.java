package frc.robot.utils;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants.ConstantsPorts;

public class LedLiason {

    public static DigitalOutput alliancePin = new DigitalOutput(ConstantsPorts.ledAlliancePin);
    public static DigitalOutput readyToFirePin = new DigitalOutput(ConstantsPorts.ledReadyToFirePin);

    /**
     * Set the alliance color
     * @param alliance 'r' for red and 'b' for b
     */
    public static void setAlliance(char alliance) {
        // Set the alliance pin. This is high if "r" and low if "b" or anything else.
        alliancePin.set(!(alliance == 'r'));
    }

    /**
     * Set the ready to fire LEDs
     * @param readyToFire True if we're ready to fire, false if we're not
     */
    public static void setReadyToFire(boolean readyToFire) {
        readyToFirePin.set(readyToFire);
    }
    
}
