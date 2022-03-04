package frc.robot.Triggers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerPOV extends Trigger {

    public enum POVDirection{
        kUp(0),
        kRight(90),
        kDown(180),
        kLeft(270),
        kNeutral(-1);

        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
    
        POVDirection(int value) {
          this.value = value;
        }
    }
    XboxController controller;
    POVDirection POV;

    /**
     * Trigger for the POV button on an XboxController. Takes a XboxController and a
     * POV direction. When the controller's POV matches the given POV, this trigger will
     * be "triggered".
     * 
     * @param controller the controller to pull the POV from
     * @param POV the POVDirection you want this trigger to activate on
     */
    public TriggerPOV(XboxController controller,  POVDirection POV) {
        super();
        this.controller = controller;
        this.POV = POV;
    }

    @Override
    public boolean get() {
        if ((controller.getPOV() < (POV.value + 5)) && (controller.getPOV() > (POV.value - 5))) { 
            return true;
        } else {
            return false;
        }
    }

    
}