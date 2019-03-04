#ifndef UR10_HPP
#define UR10_HPP

#include <vector>

#pragma pack(push, 1)
typedef struct _Ur10RealTimePacket {
    //    int     size;
    //    double	timestamp;
    //    double	target_q[6];
    //    double	target_qd[6];
    //    double	target_qdd[6];
    //    double	target_current[6];
    //    double	target_moment[6];
    //    double	actual_q[6];
    //    double	actual_qd[6];
    //    double	actual_current[6];
    //    double	joint_control_output[6];
    //    double	actual_TCP_pose[6];
    //    double	actual_TCP_speed[6];
    //    double	actual_TCP_force[6];
    //    double	target_TCP_pose[6];
    //    double	target_TCP_speed[6];
    //    unsigned long long	actual_digital_input_bits;
    //    double	joint_temperatures[6];
    //    double	actual_execution_time;
    //    int	robot_mode;
    //    int	joint_mode[6];
    //    int	safety_mode;
    //    double	actual_tool_accelerometer[3];
    //    double	speed_scaling;
    //    double  target_speed_fraction;
    //    double	actual_momentum;
    //    double	actual_main_voltage;
    //    double	actual_robot_voltage;
    //    double	actual_robot_current;
    //    double	actual_joint_voltage[6];
    //    unsigned long long actual_digital_output_bits;
    //    unsigned int runtime_state;
    //    double	elbow_position[3];
    //    double	elbow_velocity[3];
    //    unsigned int robot_status_bits;
    //    unsigned int  safety_status_bits;
    //    unsigned int analog_io_types;
    //    double standard_analog_input0;
    //    double standard_analog_input1;
    //    double standard_analog_output0;
    //    double standard_analog_output1;
    //    double io_current;
    //    unsigned int euromap67_input_bits;
    //    unsigned int euromap67_output_bits;
    //    double euromap67_24V_voltage;
    //    double euromap67_24V_current;
    //    unsigned int tool_mode;
    //    unsigned int tool_analog_input_types;
    //    double tool_analog_input0;
    //    double tool_analog_input1;
    //    int tool_output_voltage;
    //    double tool_output_current;
    //    double tool_temperature;
    //    double tcp_force_scalar;
    //    unsigned int output_bit_registers0_to_31;
    //    unsigned int output_bit_registers32_to_63;
    //    int output_int_register_X;
    //    double output_double_register_X;
    //    unsigned int input_bit_registers0_to_31;
    //    unsigned int input_bit_registers32_to_63;
    //    unsigned int input_int_register_x[24];
    //    double input_double_register_x[24];
    //    unsigned char tool_output_mode;
    //    unsigned char tool_digital_output1_mode;
    int     m_dMsgSize;                     // Total message length in bytes ---------------------------------- 0
    double	m_fTime;                        // Time elapsed since the controller was started ------------------ 4
    double	m_fArrQTarget[6];				// Target Joint Positions ----------------------------------------- 12
    double	m_fArrQDTarget[6];				// Target Joint Velocities ---------------------------------------- 60
    double	m_fArrQDDTarget[6];				// Target Joint Acceleration -------------------------------------- 108
    double	m_fArrITarget[6];				// Target Joint Current ------------------------------------------- 156
    double	m_fArrMTarget[6];				// Target Joint moments(torques) ---------------------------------- 204
    double	m_fArrQActual[6];				// Actual Joint Positions ----------------------------------------- 252  *
    double	m_fArrQDActual[6];				// Actual Joint Velocities ---------------------------------------- 300  *
    double	m_fArrIActual[6];				// Actual Joint Currents ------------------------------------------ 348
    double	m_fArrIControl[6];				// Joint control currents ----------------------------------------- 396
    double	m_fArrToolVectorActual[6];		// Tool Vector Actual (actual Cartesian coordinates of the tool) -- 444  *
    double	m_fArrTCPSpeedActual[6];		// Actual Speed of the tool given in Cartesian coordinates. ------- 492  *
    double	m_fArrTCPForce[6];				// Generalised Forces in the TCP ---------------------------------- 540
    double	m_fArrToolVectorTarget[6];		// Target Cartesian coordinates of the tool ----------------------- 588  *
    double	m_fArrTCPSpeedTarget[6];		// Target speed of the tool given in Cartesian coordinates -------- 636  *
    double	m_fDigitalInputBits;			// Current State of the digital inputs ---------------------------- 684
    double	m_fArrMotorTemperatures[6];		// Temperature of each joint motor in degrees celsius ------------- 692
    double	m_fControllerTimer;				// Controller realtime thread execution time ---------------------- 740
    double	m_fReserved0;					// A value used by Universal Robots software only ----------------- 748
    double	m_fRobotMode;					// Robot mode ----------------------------------------------------- 756
    double	m_fArrJointModes[6];			// Joint Control modes -------------------------------------------- 764
    double	m_fSafetyMode;					// Safety mode ---------------------------------------------------- 812
    double	m_fArrReserved0[6];				// Used by Universal Robots software only ------------------------- 820
    double	m_fArrToolAccelerometerValues[3];	// Tool X, y, z accelerometer values(ver 1.7) ----------------- 868  *
    double  m_fArrReserved1[6];				// Used by Universal Robots software only ------------------------- 892
    double	m_fSpeedScaling;				// Speed scaling of trajectory limiter ---------------------------- 940
    double	m_fLinearMomentumNorm;			// Norm of Cartesian linear momentum ------------------------------ 948
    double	m_fReserved1;					// A value used by Universal Robots software only ----------------- 956
    double	m_fReserved2;					// A value used by Universal Robots software only ----------------- 964
    double	m_fVMain;						// Masterboard Main Voltage --------------------------------------- 972
    double	m_fVRobot;						// Masterboard Robot Voltage(48V) --------------------------------- 980
    double	m_fIRobot;						// Masterboard Robot Current -------------------------------------- 988
    double	m_fArrVActual[6];				// Actual joint voltages ------------------------------------------ 996
    double	m_fDigitalOutputs;				// Digital Outputs ------------------------------------------------ 1044
    double	m_fProgramState;				// ProgramState --------------------------------------------------- 1052
    double  m_fElbowPos;					// Elbow pos ------------------------------------------------------ 1060
    double  m_fElbowVel;                    // Elbow vel ------------------------------------------------------ 1084
} Ur10RealTimePacket;
#pragma pack(pop)

class UR10 {
public:
    UR10() {}
    ~UR10() {}

public:
    void setBuffer(std::vector<char> buffer);
    Ur10RealTimePacket getPacket();

    static Ur10RealTimePacket convert(std::vector<char> buffer);
    static const int PACKET_SIZE = 1108;
private:
    std::vector<char> buffer;
    Ur10RealTimePacket packet;
};


#endif // UR10_HPP
