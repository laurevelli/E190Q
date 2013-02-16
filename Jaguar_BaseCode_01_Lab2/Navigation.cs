using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double diffEncoderPulseL, diffEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 50;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double maxVelocity = 0.10;
        const double Kpho = 2;
        const double Kalpha = 4;
        const double Kbeta = -1.0;//-1;
        const double alphaTrackingAccuracy = 0.05;
        const double betaTrackingAccuracy = 0.05;
        const double phoTrackingAccuracy = 0.10;
        const double maxTickSpeed = 802;
        // max speed 2.36 [m/s]*(1/(wheelRadius[m]*2*pi))*190[ticks/rev]
        double time = 0;
        DateTime startTime;


        // FSM globals for Odometry Testing:
        public bool buttonClicked = false;
        public int state = 0;
        public double[] dataPoints = { 0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 10.0 };  // for forward motion
                                    // = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0}; // for forward motion in detail
                                    // = {0.0, .628319, 1.25664, 1.88496, 2.51327, 3.14159, 3.76991, 4.39823, 5.02655, 5.65487, 6.28319};  // one rotation at 1/10th rotation per increment.
        double distanceBtwnDPs = 0;

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            this.Initialize();

            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            //InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;


            // for odometry Testing:
            // state = 0;
            buttonClicked = false;
            // start driving:


        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {
                    // check the FSM:
                    FSM();

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to 1meter from the wall. Otherwise, comment it out after lab 1. 
                    // WallPositioning();

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    ActuateMotors();


                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }

        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;
                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                lastEncoderPulseL = currentEncoderPulseL;
                lastEncoderPulseR = currentEncoderPulseR;
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();
            }
            else
            {
                // Get most recent encoder measurements
                try
                {
                    lastEncoderPulseL = currentEncoderPulseL;
                    lastEncoderPulseR = currentEncoderPulseR;
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotors()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
 //               String newData = time.ToString() + " " + x.ToString() + " " + y.ToString() + " " + t.ToString() + "dT " + distanceTravelled.ToString() + "aT " + angleTravelled.ToString() + "DP " + distanceBtwnDPs.ToString() + " " + dataPoints[state] + " " + diffEncoderPulseL.ToString();
                String newData = x.ToString() + " " + y.ToString() + " " + t.ToString() + " " + distanceBtwnDPs.ToString();// currentEncoderPulseL.ToString() + " " + currentEncoderPulseR.ToString();
                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            long centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            double LrotationsPerSec = ((currentEncoderPulseL - lastEncoderPulseL) / pulsesPerRotation) / deltaT;
            double RrotationsPerSec = ((currentEncoderPulseR - lastEncoderPulseR) / pulsesPerRotation) / deltaT;
            double avgAngularSpeed = ((LrotationsPerSec + RrotationsPerSec) * 2 * 3.14159) / 2; // [radians/sec]
            double avgLinearSpeed = avgAngularSpeed * wheelRadius;    // meters per second.

            long desiredDist = 1000;    // [mm]
            double error = (centralLaserRange - desiredDist)/1000.0; // [m]
            short minVal = 10;  // [motorInput]
            short Pconstant = 20;

            // error should be a range around 0.0 to 1.0;

            // Send Control signals, put negative on left wheel control
            
            
            motorSignalR = (short)(error*Pconstant); //desiredWheelSpeedL
            motorSignalL = (short)(error*Pconstant); //desiredWheelSpeedL

            // Add a constant offset Value:
            if (error > 0)
            {
                motorSignalR += minVal;
                motorSignalL += minVal;
            }
            else
            {
                motorSignalR -= minVal;
                motorSignalL -= minVal;
            }
            
            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // ****************** Additional Student Code: End   ************
        }



        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {

        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.
           
            // Calculate Encoder Differences:
            diffEncoderPulseR = currentEncoderPulseR - lastEncoderPulseR;
            diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;

            // Check for Overflow and take the "inverted" difference if overflow occurred.
            if (diffEncoderPulseR < (-1*maxTickSpeed) )
                diffEncoderPulseR = currentEncoderPulseR + (encoderMax - lastEncoderPulseR);
            if (diffEncoderPuleR > maxTickSpeed)
                diffEncoderPulseR = -1*(lastEncoderPulseR + (encoderMax - currentEncoderPulseR));
                
            if (diffEncoderPulseL < (-1*maxTickSpeed) )
                diffEncoderPulseL = currentEncoderPulseL + (encoderMax - lastEncoderPulseL);
            if (diffEncoderPuleL > maxTickSpeed)
                diffEncoderPulseL = -1*(lastEncoderPulseL + (encoderMax - currentEncoderPulseL));

            // Calculate Linear wheel Distance travelled (in one DeltaT): [Lecture 3, Slide 16]
            wheelDistanceL = diffEncoderPulseL / pulsesPerRotation * 2 * Math.PI * wheelRadius;
            wheelDistanceR = -diffEncoderPulseR / pulsesPerRotation * 2 * Math.PI * wheelRadius;
            
            // Calculate angle traveled (in one DeltaT):
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius);
            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2.0;
            
            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Update the actual
            
            t += angleTravelled;    // add angular displacement.
            t %= (2 * Math.PI);     // remove multiple circles around unit circle.
            // fit to range of -PI to +PI:
            if (t>Math.PI)
            {   t -= (2*Math.PI);}
                //t= -((t+(Math.PI/2)-Math.PI));}
            if (t < -Math.PI)
            { t += (2 * Math.PI); }
                //t= -((t-(Math.PI/2)+Math.PI));}

            x += distanceTravelled * Math.Cos(angleTravelled);
            y += distanceTravelled * Math.Sin(angleTravelled);

            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            x_est = x;
            y_est = y;
            t_est = t;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF




            // ****************** Additional Student Code: End   ************

        }

        public void FSM()
        {
            distanceBtwnDPs = Math.Sqrt( (Math.Pow(x,2) + Math.Pow(y, 2)) );  // get current net distance.

            // check if we're at the next dataPoint:
            if (distanceBtwnDPs >= dataPoints[state])
            {
                
                // turn motors off:
                motorSignalL = 0;
                motorSignalR = 0;

                // log data:

                // reset x and y for the next data point:
                // x = 0; y = 0; t = 0;

                // give control back to users:
                jaguarControl.controlMode = jaguarControl.MANUAL;

                // state++;
                 
            }
            else
            {
                // turn motors back on:
                motorSignalL = 20;
                motorSignalR = 20;
            }

        }

        #endregion

    }
}
