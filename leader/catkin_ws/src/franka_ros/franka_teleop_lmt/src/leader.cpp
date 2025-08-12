//============================================================================
// Name        : leader.cpp
// Author      : Basak Gülecyüz (basak.guelecyuez@tum.de)
// Version     : April 2023
// Description : leader node for teleoperation between force dimensiion haptic devices <-> franka arm 
//============================================================================

// General
#include <assert.h>
#include <gtest/gtest.h>
#include <pthread.h>
#include <time.h>
#include <queue>
#include <string>
#include <vector>

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

// ROS
//#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <ros/package.h>
// for gripping
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/StopAction.h>

// Haptics
#include "dhdc.h"
#include "drdc.h"



////////////////////////////////////////////////////////////////
// Declared Variables for ROS
////////////////////////////////////////////////////////////////
ros::NodeHandle* n;                 // Leader ROS Node
ros::Publisher* L_Pub;              // Leader publisher
ros::Publisher* L_Pub_orientation;
ros::Subscriber* L_Sub;             // Leader subscriber
ros::Timer TimerRecorder;           // Timer for recorder thread
double TimerPeriodHaptic = 0.001;   // Comm. rate (1/T)
double CurrentTime = 0;
actionlib::SimpleActionClient<franka_gripper::MoveAction>* client_move = nullptr;  // For move
actionlib::SimpleActionClient<franka_gripper::GraspAction>* client_grasp = nullptr;  // For grasping
actionlib::SimpleActionClient<franka_gripper::StopAction>* client_stop = nullptr;  // For releasing


////////////////////////////////////////////////////////////////
// Declared Variables for Bilateral Teleoperation
////////////////////////////////////////////////////////////////

// Path to package
std::string package_path = ros::package::getPath("franka_teleop_lmt");

// Demo number for saving session
std::string demo_num;


double ws = 1.0;   // Work space scaling factor btw. haptic device and robot

// Variables to be communicated between leader and follower
std::vector<double> command(3, 0.0);        // Vm[3] to be sent to follower
std::vector<double> command_orientation(3, 0.0);        // Orientation Vm [3] to be sent to follower
std::vector<double> feedback(3, 0.0);       // Fm[3] received from follower
std::queue<std::vector<double>>feedbackQ;   // Force feedback queue
bool first_packet = 0;    // True if first force feedback is received

// Leader side Variables
double Vl[3] = {0, 0, 0};             // Leader velocity
double AngularVl[3] = {0, 0, 0};      // Leader angular velocity
double Xl[3] = {0.25, 0.25, 0.4};     // Leader position
double Fl[3] = {0,0,0};               // Leader force

// Variables for Kalman filtering of Velocity 
struct KalmanState {
  double CurrentEstimation[3] = {0, 0, 0};
  double PreviousEstimation[3] = {0, 0, 0};
  double PredictionErrorVar[3] = {0, 0, 0};
};
KalmanState kalman_state;
bool FlagVelocityKalmanFilter = 1;

// Variables for Kalman filtering of Angular Velocity
KalmanState kalman_state_angular;
bool FlagAngularVelocityKalmanFilter = 1;


// Data recording
typedef struct {
  double time;
  bool first_packet;
  double Xl[3];
  double Vl[3];
  double AngularVl[3];
  double Fl[3];
} RecordData;         // Record data struct
std::queue<RecordData> RecordQueue;   // Queue what to record 
FILE* fidRecord;      // File for saving

// Fast and slow modes
enum class SpeedMode { SLOW, FAST };

std::mutex recordMutex;               // Mutex for preventing data overwriting in record
std::mutex feedbackMutex;             // Mutex for preventing data overwriting in subscriber
////////////////////////////////////////////////////////////////
// Declared Functions for Haptics
////////////////////////////////////////////////////////////////
void* HapticsLoop(void* ptr);                // Main haptics thread 
void RecordSignals(const ros::TimerEvent&);  // Thread to record the signals
void ForceCallback(const std_msgs::Float64MultiArrayConstPtr&);  // Subscriber callback
void ApplyKalmanFilter_Velocity(double* CurrentSample);    // Kalman fitler velocity
void ApplyKalmanFilter_AngularVelocity(double* CurrentSample);    // Kalman fitler angular velocity
double getSpeedValue(SpeedMode mode);
bool handleSpeedModeSwitch(bool button_pressed,
  SpeedMode& current_mode,
  ros::Time& button_press_time,
  bool& button_was_pressed,
  bool& mode_switched,
  double& speed_scale);
void keepLargestAngularComponent(double angularVel[4]);
int InitHaptics();  // Haptic device initialization


int main(int argc, char** argv) {


  ////////////////////////////////////////////////////////////////
  // Initialize ROS
  ////////////////////////////////////////////////////////////////

  ros::init(argc, argv, "leader");
  n = new ros::NodeHandle;
  

  // Demo number
  demo_num= argv[1];

  // Leader publisher for commands
  L_Pub = new ros::Publisher;
  *L_Pub = n->advertise<std_msgs::Float64MultiArray>("/cartesian_impedance_example_controller/LFcommand", 1, false);


  L_Pub_orientation = new ros::Publisher;
  *L_Pub_orientation = n->advertise<std_msgs::Float64MultiArray>("/cartesian_impedance_example_controller/OFcommand", 1, false);


  // Grasping
  client_grasp = new actionlib::SimpleActionClient<franka_gripper::GraspAction>("franka_gripper/grasp", true);
  client_grasp->waitForServer();

  // Moving
  client_move = new actionlib::SimpleActionClient<franka_gripper::MoveAction>("franka_gripper/move", true);
  client_move->waitForServer();

  // Releasing
  client_stop = new actionlib::SimpleActionClient<franka_gripper::StopAction>("/franka_gripper/stop", true);

  // Open the gripper to initial width
  franka_gripper::MoveGoal init_gripper;
  init_gripper.width = 0.08;
  init_gripper.speed = 0.05;
  client_move->sendGoal(init_gripper);

  // Leader subscriber for feedback
  L_Sub = new ros::Subscriber;
  *L_Sub = n->subscribe<std_msgs::Float64MultiArray>(
      "/cartesian_impedance_example_controller/FLfeedback", 1, &ForceCallback, ros::TransportHints().tcpNoDelay(true));

  // Sleep for 5 seconds 
  ros::Duration(5.0).sleep();
  ROS_INFO("Publisher & Subscriber are set! ");

  ////////////////////////////////////////////////////////////////
  // Initialize Recording
  ////////////////////////////////////////////////////////////////

  std::string record_path =  package_path + "/TeleopData/leader_" + demo_num + ".txt"; 
  fidRecord = fopen(record_path.c_str(), "w");
  if (fidRecord== NULL) {
    ROS_INFO("Failed: ");
    return 1;
}
  fprintf(fidRecord, "time\t");                // timestamp
  fprintf(fidRecord, "Vl_x\tVl_y\tVl_z\t");    // Leader Velocity
  fprintf(fidRecord, "Xl_x\tXl_y\tXl_z\t");    // Leader Position
  fprintf(fidRecord, "Fl_x\tFl_y\tFl_z\t");  // Leader Force
	fprintf(fidRecord, "first_packet\n"); // first packet received
  fclose(fidRecord);
  TimerRecorder = n->createTimer(ros::Duration(TimerPeriodHaptic),
                                 RecordSignals);  // create a timer callback for data record

  ////////////////////////////////////////////////////////////////
  // Initialize Haptics
  ////////////////////////////////////////////////////////////////

  // Initialize the haptic device
  InitHaptics();

  ros::Duration(5.0).sleep();
  ROS_INFO("Haptic Device is initialized! ");


  ////////////////////////////////////////////////////////////////
  // Main haptics loop @1 kHz 
  ////////////////////////////////////////////////////////////////

  pthread_t HapticsLoop_t;
  pthread_create(&HapticsLoop_t, NULL, HapticsLoop, NULL);
  pthread_join(HapticsLoop_t, NULL);

  ////////////////////////////////////////////////////////////////
  // Close everything safely
  ////////////////////////////////////////////////////////////////

  return 0;
}

void* HapticsLoop(void* ptr) {

  // Enable force on Haptic Device
  dhdEnableForce (DHD_ON);

  // Loop at 1 kHz
  ros::Rate loop_rate(1000);
  ros::AsyncSpinner spinner(3);
  spinner.start();

  while (ros::ok()) {

  // 1. Get velocity from haptic device
  // ---> Kalman filtering on velocity to remove high frequency fluctuations
  // 2. Receive force from follower 
  // ---> Additional damping on force to stabilize
  // 3. Display forces to haptic device
  // 4. Publish velocity to follower
  // 5. Push into RecordQueue for saving

    /////////////////////////////////////////////////////////
    // TO-DO: 1. Get velocity & position from haptic device
    /////////////////////////////////////////////////////////
    
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Your code here!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    dhdGetLinearVelocity(&Vl[0], &Vl[1], &Vl[2]);
    dhdGetLinearVelocity(&AngularVl[0], &AngularVl[1], &AngularVl[2]);

    // Get button states
    bool button0 = (dhdGetButton(0) == 1);  // button 0
    bool button1 = (dhdGetButton(1) == 1);  // button 1
    bool button2 = (dhdGetButton(2) == 1);  // button 2
    bool button3 = (dhdGetButton(3) == 1);  // button 3
    static bool last_button2 = false;
    static bool last_button3 = false;

    // Variables to switch between slow and fast mode
    static SpeedMode current_mode = SpeedMode::FAST;  // Initialization to fast mode
    static ros::Time button_press_time;
    static bool button_was_pressed = false;
    static bool mode_switched = false;
    static double speed_scale = 0.0;

    bool allow_motion = handleSpeedModeSwitch(button0, current_mode, button_press_time, button_was_pressed, mode_switched, speed_scale);

    //  workspace scaling
    for (int i = 0; i < 3; ++i) {
      if (allow_motion) {
        Vl[i] = Vl[i] * speed_scale;
      }
      else {
        Vl[i] = Vl[i] * 0.0;
      }
    }

    // Orientation Mode
    for (int i = 0; i < 3; ++i) {
      if (button1) {
        AngularVl[i] = AngularVl[i] * 5;
      }
      else {
        AngularVl[i] = AngularVl[i] * 0.0;
      }
    }

    
    /////////////////////////////////////////////////////////
    // ---> Kalman filtering on velocity
    /////////////////////////////////////////////////////////

    if (FlagVelocityKalmanFilter == 1) {
      ApplyKalmanFilter_Velocity(Vl);
      Vl[0] = kalman_state.CurrentEstimation[0];
      Vl[1] = kalman_state.CurrentEstimation[1];
      Vl[2] = kalman_state.CurrentEstimation[2];
    }

    if (FlagAngularVelocityKalmanFilter == 1) {
      ApplyKalmanFilter_AngularVelocity(AngularVl);
      AngularVl[0] = kalman_state_angular.CurrentEstimation[0];
      AngularVl[1] = kalman_state_angular.CurrentEstimation[1];
      AngularVl[2] = kalman_state_angular.CurrentEstimation[2];
    }

    keepLargestAngularComponent(AngularVl);


    // Update Leader position
    for (int i = 0; i < 3; ++i) {
      Xl[i] = Xl[i] + Vl[i] * 0.001;
    }

	  /////////////////////////////////////////////////////////////////////
		// TO-DO: 2. Check receive queues for force feedback
		/////////////////////////////////////////////////////////////////////
  if (feedbackMutex.try_lock()) {
      if (feedbackQ.size()) {
        
        // Get the front value from feedbackQ into variable feedback.
        // Pop from queue feedbackQ
        /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Your code here!
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
    
        std::vector<double> store(3, 0.0);
        store = feedbackQ.front();
        feedbackQ.pop();

        // Copy the vector feedback into Fl
        /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Your code here!
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

        for (int i = 0; i < 3; ++i) {
          Fl[i] = store[i]; 
        }

      }
    feedbackMutex.unlock(); 
  }

		/////////////////////////////////////////////////////////////////////
		//--->  Damping on Force for stability
		/////////////////////////////////////////////////////////////////////

    // Extra Damping
    double damping_factor = 0.5;
    double Fl_norm = 0;
    for (int i = 0; i < 3; ++i) {
        Fl_norm += Fl[i] * Fl[i] ;
      }
    Fl_norm = sqrt(Fl_norm);

    if(Fl_norm > 2.0){
        // ROS_INFO("Damping! ");
        for (int i = 0; i < 3; ++i) {
          Fl[i] = Fl[i] + damping_factor*Vl[i]/ws;
        }
    }

    /////////////////////////////////////////////////////////
    // TO-DO 3. Display forces to haptic device
    /////////////////////////////////////////////////////////

    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Your code here!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    // dhdSetForce(0, 0, 0);

    dhdSetForce(-Fl[0]/4, -Fl[1]/4, -Fl[2]/4);

    // ROS_INFO_STREAM("Deployed Forces: ["
		// << -Fl[0]/4 << ", "
		// << -Fl[1]/4 << ", "
		// << -Fl[2]/4 << "]");

    /////////////////////////////////////////////////////////
    // TO-DO: 4. Publish command
    /////////////////////////////////////////////////////////

    // Send new command
    /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Your code here!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    // Velocity message
    std_msgs::Float64MultiArray msg_velocity;
    command.clear();
    command.insert(command.end(), Vl, Vl + 3);
    msg_velocity.data = command;
    L_Pub->publish(msg_velocity);

    // Angular velocity message
    std_msgs::Float64MultiArray msg_velocity_orientation;
    command_orientation.clear();
    
    command_orientation.insert(command_orientation.end(), AngularVl, AngularVl + 3);
    msg_velocity_orientation.data = command_orientation;
    L_Pub_orientation->publish(msg_velocity_orientation);

    // Control the gripper
    constexpr double kStep = 0.005;
    constexpr double kMaxWidth = 0.08;
    constexpr double kMinWidth = 0.0;

    static double gripper_width = kMaxWidth;  // Initial width is 0.08m
    constexpr double kGripperSpeed = 0.05;  // Gripper speed

    // Grasp the object
    if (button2 && button3) {
      std::cout << "grasp" << std::endl;
      franka_gripper::GraspGoal grasp_goal;
      grasp_goal.width = gripper_width - kStep;
      grasp_goal.speed = kGripperSpeed;
      grasp_goal.force = 20;
      grasp_goal.epsilon.inner = 0.05;
      grasp_goal.epsilon.outer = 0.05;

      client_grasp->sendGoal(grasp_goal);
    }

    // Close the gripper
    else if (button2 && !last_button2) {
      gripper_width = std::max(kMinWidth, gripper_width - kStep);

      std::cout << "close" 
      << "width" << gripper_width
      << std::endl;

      franka_gripper::MoveGoal move_goal;
      move_goal.width = gripper_width;
      move_goal.speed = kGripperSpeed;

      client_move->sendGoal(move_goal);
    }

    // Open the gripper
    else if (button3 && !last_button3) {
      gripper_width = std::min(kMaxWidth, gripper_width + kStep);

      std::cout << "open " 
      << "width" << gripper_width
      << std::endl;

      franka_gripper::MoveGoal move_goal;
      move_goal.width = gripper_width;
      move_goal.speed = kGripperSpeed;

      client_move->sendGoal(move_goal);
    }

    last_button2 = button2;
    last_button3 = button3;

    
    /////////////////////////////////////////////////////////
    // 5. Record Signals for analysis?/+=-/'\]=+
    /////////////////////////////////////////////////////////

    RecordData* tmp = new RecordData;  // temporary variable storing the signals

    // time
    tmp->time = CurrentTime;


    // Leader states
    for (int k = 0; k < 3; k++) {
      tmp->Vl[k] = Vl[k];
      tmp->Xl[k] = Xl[k];
      tmp->Fl[k] = Fl[k];
  
    }

		// First packet received
		tmp->first_packet = first_packet;
    
    // Push to record queue
    if (recordMutex.try_lock()) {
      RecordQueue.push(*tmp);
      delete tmp;
      recordMutex.unlock(); 
    }


    CurrentTime = CurrentTime + TimerPeriodHaptic;
    loop_rate.sleep();

    }
  return NULL;
}

void RecordSignals(const ros::TimerEvent&) {

  std::string record_path =  package_path + "/TeleopData/leader_" + demo_num + ".txt"; 
  fidRecord = fopen(record_path.c_str(), "a");
  if (fidRecord == NULL) {
    ROS_INFO("File error");
  }
  bool DataArrived = false;
  RecordData* tmp = new RecordData;

  const std::lock_guard<std::mutex> lock(recordMutex);
  //ROS_INFO("Mutex Lock Record Queue 2! ");
  if (RecordQueue.size() > 0) {

    *tmp = RecordQueue.front();
    RecordQueue.pop();

    DataArrived = true;
  } else {
    DataArrived = false;
  }
  if (DataArrived == true) {
    fprintf(fidRecord, "%f\t", tmp->time);
    fprintf(fidRecord, "%f\t%f\t%f\t", tmp->Vl[0], tmp->Vl[1], tmp->Vl[2]);
    fprintf(fidRecord, "%f\t%f\t%f\t", tmp->Xl[0], tmp->Xl[1], tmp->Xl[2]);
    fprintf(fidRecord, "%f\t%f\t%f\t", tmp->Fl[0], tmp->Fl[1], tmp->Fl[2]);
		fprintf(fidRecord, "%d\n", tmp->first_packet);
  }
  fclose(fidRecord);
}


void ForceCallback(const std_msgs::Float64MultiArrayConstPtr& msg) {

const std::lock_guard<std::mutex> lock(feedbackMutex);
  // Receive feedback into queue
  feedbackQ.push(msg->data);

	if(feedbackQ.size()>0){
		first_packet = 1;
	}

}

void ApplyKalmanFilter_Velocity(double* CurrentSample) {
  double Innovation[3];     // I
  double InnovationVar[3];  // S
  double NoiseVar[3];       // R
  double Gain[3];           // K
  double ProcNoiseVar[3];   // Q

  NoiseVar[0] = 2000.0;
  NoiseVar[1] = 2000.0;
  NoiseVar[2] = 2000.0;

  ProcNoiseVar[0] = 1.0;
  ProcNoiseVar[1] = 1.0;
  ProcNoiseVar[2] = 1.0;

  for (int i = 0; i < 3; i++) {
    Innovation[i] = CurrentSample[i] - kalman_state.PreviousEstimation[i];
    InnovationVar[i] = kalman_state.PredictionErrorVar[i] + NoiseVar[i];
    Gain[i] = kalman_state.PredictionErrorVar[i] / InnovationVar[i];

    kalman_state.CurrentEstimation[i] =
        kalman_state.PreviousEstimation[i] + Gain[i] * Innovation[i];

    // update values for next iteration
    kalman_state.PredictionErrorVar[i] = kalman_state.PredictionErrorVar[i] + ProcNoiseVar[i] -
                                         Gain[i] * kalman_state.PredictionErrorVar[i];
    kalman_state.PreviousEstimation[i] = kalman_state.CurrentEstimation[i];
  }
}

void ApplyKalmanFilter_AngularVelocity(double* CurrentSample) {
  double Innovation[3];     // I
  double InnovationVar[3];  // S
  double NoiseVar[3];       // R
  double Gain[3];           // K
  double ProcNoiseVar[3];   // Q

  NoiseVar[0] = 2000.0;
  NoiseVar[1] = 2000.0;
  NoiseVar[2] = 2000.0;

  ProcNoiseVar[0] = 1.0;
  ProcNoiseVar[1] = 1.0;
  ProcNoiseVar[2] = 1.0;

  for (int i = 0; i < 3; i++) {
    Innovation[i] = CurrentSample[i] - kalman_state_angular.PreviousEstimation[i];
    InnovationVar[i] = kalman_state_angular.PredictionErrorVar[i] + NoiseVar[i];
    Gain[i] = kalman_state_angular.PredictionErrorVar[i] / InnovationVar[i];

    kalman_state_angular.CurrentEstimation[i] =
        kalman_state_angular.PreviousEstimation[i] + Gain[i] * Innovation[i];

    // update values for next iteration
    kalman_state_angular.PredictionErrorVar[i] = kalman_state_angular.PredictionErrorVar[i] + ProcNoiseVar[i] -
                                         Gain[i] * kalman_state_angular.PredictionErrorVar[i];
    kalman_state_angular.PreviousEstimation[i] = kalman_state_angular.CurrentEstimation[i];
  }
}

double getSpeedValue(SpeedMode mode) {
  return (mode == SpeedMode::FAST) ? 5.0 : 2.0;
}

bool handleSpeedModeSwitch(
    bool button_pressed,
    SpeedMode& current_mode,
    ros::Time& button_press_time,
    bool& button_was_pressed,
    bool& mode_switched,
    double& speed_scale) {
  
  ros::Duration press_duration;

  if (button_pressed && !button_was_pressed) {
    // First cycle, record press time
    button_press_time = ros::Time::now();
    mode_switched = false;
    button_was_pressed = true;
  } 
  
  else if (button_pressed && button_was_pressed) {
    // Button being held
    press_duration = ros::Time::now() - button_press_time;

    if (press_duration.toSec() > 0.3) {
      // Long press → enable movement
      speed_scale = getSpeedValue(current_mode);
      return true;
    }
  }
  
  else if (!button_pressed && button_was_pressed) {
    // Button just released
    press_duration = ros::Time::now() - button_press_time;

    if (press_duration.toSec() <= 0.3 && !mode_switched) {
      // Short press → toggle mode
      current_mode = (current_mode == SpeedMode::SLOW)
                     ? SpeedMode::FAST
                     : SpeedMode::SLOW;

      std::cout << "Switched to: "
                << ((current_mode == SpeedMode::FAST) ? "FAST" : "SLOW")
                << std::endl;

      mode_switched = true;
    }

    button_was_pressed = false;
  }

  return false;  // Movement not enabled
}

void keepLargestAngularComponent(double angularVel[4]) {
    int max_idx = 0;
    float max_val = std::abs(angularVel[0]);

    for (int i = 1; i < 3; ++i) {
        if (std::abs(angularVel[i]) > max_val) {
            max_val = std::abs(angularVel[i]);
            max_idx = i;
        }
    }

    for (int i = 0; i < 3; ++i) {
        if (i != max_idx)
            angularVel[i] = 0.0f;
    }
}


// haptic device initialization
int InitHaptics() {
  if (dhdOpen() >= 0) {
    printf("%s device detected\n", dhdGetSystemName());
  }

  else {
    printf("no device detected\n");
    dhdSleep(2.0);
    exit(0);
  }
  printf("\n");

  return 0;
}
