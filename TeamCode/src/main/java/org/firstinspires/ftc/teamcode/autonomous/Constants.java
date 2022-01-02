package org.firstinspires.ftc.teamcode.autonomous;

public final class Constants {
    //This class is home to all of the drive and field constants that are used multiple times in the code.
    public static final String VUFORIA_KEY = "AS0ENI3/////AAABmRrhaZtkGkSMi4tGQFf9azI3tZlg7Xv8GCAFy/EtV7oDQmsVBBNgiQNq035C7ShFgSt1Y9dtgOUrPHhlgoI/8sqhoBUnr3WRm/ex/gPsScPYlpy4mqBUZEIQxI2hndIuFrxPSc5gCMC4kyay2RWUWthzUygnp/22kgrq2u7xyKLwsUIctziWB1T3xreY6LcdSuqgPx6qMeiOmPkqLrIm+BbJovtmoVA7d/PqPoIeoo6O/CurFZVUeJq7zkPRB9OzsoF3Iyxyd3jGi1xlPes828QsbIcx1UYQIyR+q52fLVAt69FPPQ6AO8YMfgc0z+qF7pSA1Vee1LIyF+HCMh67gXj3YntVhvlnSeflrFtVB7vl";
    public static final double DISTANCE_PER_TICK = 0.701248; //5202 Series Motor (PPR)
    public static double INIT_THETA = 0; //Value used to subtract from gyro to fix error. This  value is temporary until root problem can be solved.
    public static Boolean IS_LEFT_OPMODE = false;
    public static Boolean IS_BLUE_TEAM = false;
    public static final double LEFT_INITIAL_Y = 2083;
    public static final double RIGHT_INITIAL_Y = 889;
    public static final double INITIAL_X = 229;
    public static final double BLUE_INITIAL_THETA = 3 * Math.PI/2;
    public static final double RED_INITIAL_THETA = Math.PI/2;
    public static final double FIELD_LENGTH = 3657.6;

}
