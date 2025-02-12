package frc.robot.constants;

import java.util.Dictionary;
import java.util.Hashtable;

public class AutoConstants {
    public static Dictionary<String, String> autoList = new Hashtable<String, String>() {{
        put("InstantCommand", "No Auto Selected");
        put("Left Auto", "3Cor 2-0-0-1, 1Alg | J1-LS-L1-LS-L4");
        put("Right Auto", "3Cor 2-0-0-1, 1Alg | E1-RS-C1-RS-C4");
        put("Center Auto", "3Cor 2-0-0-1, 1Alg | G1");
    }};
}
