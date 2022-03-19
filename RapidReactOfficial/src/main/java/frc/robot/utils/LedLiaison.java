package frc.robot.utils;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.Constants.ConstantsPorts;

public class LedLiaison {

    // Instantiate all LED pin outputs
    private static DigitalOutput connected = new DigitalOutput(ConstantsPorts.ledsConnectedPin);
    private static DigitalOutput allianceColor = new DigitalOutput(ConstantsPorts.ledsAlliancePin);
    private static DigitalOutput cargoSeen = new DigitalOutput(ConstantsPorts.ledsCargoSeenPin);
    private static DigitalOutput readyToFire = new DigitalOutput(ConstantsPorts.ledsReadyToFirePin);

    /**
     * Set whether we're connected (code is running)
     * @param value Whether we're connected (code is running)
     */
    public static void setConnected(boolean value) {
        connected.set(value);
    }

    /**
     * Set the alliance color
     * @param value The alliance color as a boolean. Low = blue, High = red
     */
    public static void setAllianceColor(boolean value) {
        allianceColor.set(value);
    }

    /**
     * Set the alliance color
     * @param color The alliance color as a char. 'b' = blue, 'r' = red
     */
    public static void setAllianceColor(char color) {
        if(color == 'b') {
            allianceColor.set(false);
        } else if(color == 'r') {
            allianceColor.set(true);
        } else {
            System.out.println("Error when setting alliance color: Color entered, " + color + ", is not a valid color!");
        }
    }

    /**
     * Set whether cargo is seen by the CargoHound
     * @param value Whether cargo is seen by the CargoHound
     */
    public static void setCargoSeen(boolean value) {
        cargoSeen.set(value);
    }

    /**
     * Set whether our shooter is ready to fire
     * @param value Whether the shooter is ready to fire
     */
    public static void setReadyToFire(boolean value) {
        readyToFire.set(value);
    }
    
}
