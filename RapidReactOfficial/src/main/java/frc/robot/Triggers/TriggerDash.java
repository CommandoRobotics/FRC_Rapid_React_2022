package frc.robot.Triggers;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerDash extends Trigger {

    NetworkTableEntry ntEntry;
    NetworkTableType ntType;
    NetworkTableValue previousValue;
    boolean alreadyNotified;
    Boolean activateCondition;
    Double activateNumber;
    String activateString;

    /**
     * Trigger for listening for a change in a NetworkTableEntry and scheduling a command. 
     * By default this trigger will activate when the value contained in the given Entry
     * is changed in any form. This only activates once, and will not "stay" on.
     * @param entryToWatch
     */
    public TriggerDash(NetworkTableEntry entryToWatch) {
        super();
        ntEntry = entryToWatch;
        previousValue = ntEntry.getValue();
    } 

    /**
     * Trigger for listening for a boolean in a NetworkTableEntry and scheduling a command. 
     * This constructor takes an activation condition to listen for. If the given Entry is ever
     * equal to the activateCondition, this trigger will be activated. This is continuous, and will
     * continue to be activated as long as the Entry matches the activateCondition 
     * @param entryToWatch
     * @param activateCondition
     */
    public TriggerDash(NetworkTableEntry entryToWatch, boolean activateCondition) {
        super();
        ntEntry = entryToWatch;
        this.activateCondition = activateCondition;
        previousValue = ntEntry.getValue();
    }

    /**
     * Trigger for listening for a value in a NetworkTableEntry and scheduling a command. 
     * This constructor takes an activation number to listen for. If the given Entry is ever
     * equal to the activateNumber, this trigger will be activated. This is continuous, and will
     * continue to be activated as long as the Entry matches the activateNumber 
     * @param entryToWatch
     * @param activateNumber
     */
    public TriggerDash(NetworkTableEntry entryToWatch, double activateNumber) {
        super();
        ntEntry = entryToWatch;
        this.activateNumber = activateNumber;
        previousValue = ntEntry.getValue();
    }

    /**
     * Trigger for listening for a value in a NetworkTableEntry and scheduling a command. 
     * This constructor takes an activation String to listen for. If the given Entry is ever
     * equal to the activateString, this trigger will be activated. This is continuous, and will
     * continue to be activated as long as the Entry matches the activateString 
     * @param entryToWatch
     * @param activateString
     */
    public TriggerDash(NetworkTableEntry entryToWatch, String activateString) {
        super();
        ntEntry = entryToWatch;
        this.activateString = activateString;
        previousValue = ntEntry.getValue();
    }

    @Override
    public boolean get() {
        NetworkTableValue currentValue = ntEntry.getValue();
        if (currentValue.getType() != NetworkTableType.kUnassigned) { 
            if (alreadyNotified) {
                alreadyNotified = false;
                System.out.println("Entry " + ntEntry.getName() + " was assigned and will now update DashTrigger");
            }
            if (activateCondition != null) {
                if (currentValue.getBoolean() == activateCondition.booleanValue()) {
                    return true;
                }
            } else if (activateNumber != null) {
                if (currentValue.getDouble() == activateNumber.doubleValue()) {
                    return true;
                }
            } else if (activateString != null) {
                if (currentValue.getString().equals(activateString)) {
                    return true;
                }
            } else if (!currentValue.equals(previousValue)) {
                previousValue = currentValue;
                return true;
            } 
        } else if (!alreadyNotified) {
            alreadyNotified = true;
        }
        previousValue = currentValue;
        return false;
    }
}