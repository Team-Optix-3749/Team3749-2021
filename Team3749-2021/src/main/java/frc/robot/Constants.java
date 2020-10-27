package frc.robot;

import java.util.HashMap;

public final class Constants {
    private static HashMap<String, Integer> mapPWM; // the PWM port map
    private static HashMap<String, Integer> mapCAN; // the CAN port map
    private static HashMap<String, Integer> mapDIO; // the DIO port map
    private static HashMap<String, Integer> mapCTRL; // the controller port map
    private static HashMap<String, Integer> mapSys; // the subsystems mode map

    public Constants() {
        // Configure the button bindings
        mapPWM = new HashMap<>();
        mapCAN = new HashMap<>();
        mapDIO = new HashMap<>();
        mapCTRL = new HashMap<>();
        mapSys = new HashMap<>();

        // loading map values for drive
        // first character = left or right
        // second character = front, middle, or back
        setCAN("drive_lf", 11);
        setCAN("drive_lb", 20);
        setCAN("drive_rf", 13);
        setCAN("drive_rb", 23);

        // shooter and elevator map
        setCAN("shooter_motor", 10);
        setCAN("intake_motor_f", 21);
        setCAN("intake_motor_b", 22);
        setCAN("elevator_motor", 24);
        setCAN("control_panel_motor", 15);

        // whether a subsystem is installed and in use
        // 0 = disabled, 1 = enabled, 2 = enabled and debugging (print sensor vals, etc)
        setSys("drive", 1);
        setSys("shooter", 1);
        setSys("intake", 1);
        setSys("elevator", 1);
        setSys("controlPanel", 1);
        setSys("vision", 1);
    }

    /**
     * Method to set a PWM port
     * 
     * @param String name of what port is for (what you call it throughout the
     *               program)
     * @param int    the port number
     */
    public void setPWM(String name, int port) {
        mapPWM.put(name, port);
    }

    /**
     * Method to set a CAN port
     * 
     * @param String name of what port is for (what you call it throughout the
     *               program)
     * @param int    the port number
     */
    public void setCAN(String name, int port) {
        mapCAN.put(name, port);
    }

    /**
     * Method to set a DIO port
     * 
     * @param String name of what port is for (what you call it throughout the
     *               program)
     * @param int    the port number
     */
    public void setDIO(String name, int port) {
        mapDIO.put(name, port);
    }

    /**
     * Method to set a CTRL port
     * 
     * @param String name of what port is for (what you call it throughout the
     *               program)
     * @param int    the port number
     */
    public void setCTRL(String name, int port) {
        mapCTRL.put(name, port);
    }

    /**
     * Method to set a subsystem value
     * 
     * @param String name of what port is for (what you call it throughout the
     *               program)
     * @param int    the value (0 = disable, 1 = enabled, 2 = debugging)
     */
    public void setSys(String name, int val) {
        mapSys.put(name, val);
    }

    /**
     * Method to get a PWM port
     * 
     * @param String name of what port is for (what you call it throughout the
     *               program)
     */
    public int getPWM(String name) {
        return mapPWM.get(name);
    }

    /**
     * Method to get a CAN port
     * 
     * @param String name of what port is for (what you call it throughout the
     *               program)
     */
    public int getCAN(String name) {
        return mapCAN.get(name);
    }

    /**
     * Method to get a DIO port
     * 
     * @param String name of what port is for (what you call it throughout the
     *               program)
     */
    public int getDIO(String name) {
        return mapDIO.get(name);
    }

    /**
     * Method to get a controller port
     * 
     * @param String name of what port is for (what you call it throughout the
     *               program)
     */
    public int getCTRL(String name) {
        return mapCTRL.get(name);
    }

    /**
     * Method to get a toggleable setting (for subsystems mostly)
     * 
     * @param String name of setting it is for (what you call it throughout the
     *               program)
     */
    public int getSys(String name) {
        return mapSys.get(name);
    }
}
