// lcm related headfile
#include <lcm/lcm-cpp.hpp>
#include "leg_control_data_lcmt.hpp"
#include "state_estimator_lcmt.hpp"
#include "rc_command_lcmt.hpp"
#include "pd_tau_targets_lcmt.hpp"
// standard headfile
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <cmath>
// unitree_sdk2 related headfile
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/client/client.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>


// PZH:
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#define TOPIC_HIGHSTATE "rt/sportmodestate"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"

// 为保证项目代码的稳定性和易理解，没有采用unitree_sdk2中采用的using namespace语句

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);


enum test_mode
{
    /*---Basic motion---*/
    normal_stand,
    balance_stand,
    velocity_move,
    trajectory_follow,
    stand_down,
    stand_up,
    damp,
    recovery_stand,
    /*---Special motion ---*/
    sit,
    rise_sit,
    stretch,
    wallow,
    //content,
    pose,
    scrape,
    front_flip,
    front_jump,
    front_pounce,
    stop_move = 99
};


enum gait_mode
{
    idle,
    trot,
    run,
    climb_stair,
    forwardDownStair,
    adjust=9
};

// 无需更改：Unitree 提供的电机校验函数
uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}


// 遥控器键值联合体，摘自unitree_sdk2，无需更改
typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;


class Custom
{
public:
    explicit Custom(){}
    ~Custom(){}

    void Init();
    void InitLowCmd();
    void Loop();
    void LowStateMessageHandler(const void* messages);
    void JoystickHandler(const void *message);
    void InitRobotStateClient();
    void activateService(const std::string& serviceName,int activate);
    void lcm_send();
    void lcm_receive();
    void robot_control();
    void lcm_receive_Handler(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt* msg);
    void lcm_high_receive_Handler(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt* msg);
    void LowCmdWrite();
    void HighCmdWrite();
    void SetStandUp();
    int queryServiceStatus(const std::string& serviceName);

// PZH:
void HighStateHandler(const void *message)
    {
        state = *(unitree_go::msg::dds_::SportModeState_ *)message;

//        printf(state.position())

//         std::cout << "HIGHSTATE: Position: " << state.position()[0] << ", " << state.position()[1] << ", " << state.position()[2] << std::endl;
//         std::cout << "HIGHSTATE: IMU rpy: " << state.imu_state().rpy()[0] << ", " << state.imu_state().rpy()[1] << ", " << state.imu_state().rpy()[2] << std::endl;
    };

    leg_control_data_lcmt leg_control_lcm_data = {0};
    state_estimator_lcmt body_state_simple = {0};
    pd_tau_targets_lcmt joint_command_simple = {0};
    rc_command_lcmt rc_command = {0};

    unitree_go::msg::dds_::LowState_ low_state{};
    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::WirelessController_ joystick{};
    unitree::robot::go2::RobotStateClient rsc;

    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_suber;
    lcm::LCM lc;

    xKeySwitchUnion key;
    int mode = 0;
    int motiontime = 0;
    int high_motiontime = 0;
    float dt = 0.002; // unit [second]
    bool _firstRun;

    /*LowCmd write thread*/
    // DDS相关的底层命令发送线程指针
    unitree::common::ThreadPtr LcmSendThreadPtr;
    unitree::common::ThreadPtr LcmRecevThreadPtr;
    unitree::common::ThreadPtr lowCmdWriteThreadPtr;
    unitree::common::ThreadPtr HighCmdWriteThreadPtr;

// PZH: Sport client
    unitree_go::msg::dds_::SportModeState_ state;
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;


};

void Custom::InitRobotStateClient()
{
    rsc.SetTimeout(5.0f);
    rsc.Init();
}

int Custom::queryServiceStatus(const std::string& serviceName)
{
    std::vector<unitree::robot::go2::ServiceState> serviceStateList;
    int ret,serviceStatus;
    ret = rsc.ServiceList(serviceStateList);
    size_t i, count=serviceStateList.size();
    for (i=0; i<count; i++)
    {
        const unitree::robot::go2::ServiceState& serviceState = serviceStateList[i];
        if(serviceState.name == serviceName)
        {
            if(serviceState.status == 0)
            {
                std::cout << "name: " << serviceState.name <<" is activate"<<std::endl;
                serviceStatus = 1;
            }
            else
            {
                std::cout << "name:" << serviceState.name <<" is deactivate"<<std::endl;
                serviceStatus = 0;
            }
        }
    }
    return serviceStatus;

}


void Custom::activateService(const std::string& serviceName,int activate)
{
    rsc.ServiceSwitch(serviceName, activate);
}

void Custom::LowStateMessageHandler(const void* message)
{
    // 用sdk2读取的底层state
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void Custom::JoystickHandler(const void *message)
{
    // 遥控器信号
    joystick = *(unitree_go::msg::dds_::WirelessController_ *)message;
    key.value = joystick.keys();
}

// -------------------------------------------------------------------------------
// 线程 1 ： lcm send 线程
// 此线程作用：实时通过unitree_sdk2读取low_state信号和joystick信号，并发送给lcm中间件
void Custom::lcm_send(){
    // leg_control_lcm_data
    for (int i = 0; i < 12; i++)
    {
        leg_control_lcm_data.q[i] = low_state.motor_state()[i].q();
        leg_control_lcm_data.qd[i] = low_state.motor_state()[i].dq();
        leg_control_lcm_data.tau_est[i] = low_state.motor_state()[i].tau_est();
    }
    // 从IMU读取姿态信息
    for(int i = 0; i < 4; i++){
        // 四元数
        body_state_simple.quat[i] = low_state.imu_state().quaternion()[i];
    }
    for(int i = 0; i < 3; i++){
        // roll pitch yaw
        body_state_simple.rpy[i] = low_state.imu_state().rpy()[i];
        // IMU 三轴加速度
        body_state_simple.aBody[i] = low_state.imu_state().accelerometer()[i];
        // IMU 三轴线性加速度
        body_state_simple.omegaBody[i] = low_state.imu_state().gyroscope()[i];
    }
    for(int i = 0; i < 4; i++){
        // 足端触地力
        body_state_simple.contact_estimate[i] = low_state.foot_force()[i];
    }
    // 遥控器按键值和摇杆数值
    rc_command.left_stick[0] = joystick.lx();
    rc_command.left_stick[1] = joystick.ly();
    rc_command.right_stick[0] = joystick.rx();
    rc_command.right_stick[1] = joystick.ry();
    rc_command.right_lower_right_switch = key.components.R2;
    rc_command.right_upper_switch = key.components.R1;
    rc_command.left_lower_left_switch = key.components.L2;
    rc_command.left_upper_switch = key.components.L1;

    if(key.components.A > 0){
        mode = 0;
    } else if(key.components.B > 0){
        mode = 1;
    }else if(key.components.X > 0){
        mode = 2;
    }else if(key.components.Y > 0){
        mode = 3;
    }else if(key.components.up > 0){
        mode = 4;
    }else if(key.components.right > 0){
        mode = 5;
    }else if(key.components.down > 0){
        mode = 6;
    }else if(key.components.left > 0){
        mode = 7;
    }

    rc_command.mode = mode;


    lc.publish("leg_control_data", &leg_control_lcm_data);
    lc.publish("state_estimator_data", &body_state_simple);
    lc.publish("rc_command", &rc_command);

    // std::cout << "loop: message is sending ......" << std::endl;
}


// -------------------------------------------------------------------------------
// 线程 2 ： lcm receive 线程
// 此线程作用：实时通过lcm中间件读取pytorch神经网络输出的期望关节控制信号（q, qd, kp, kd, tau_ff）
// 查看 go2_gym_deploy/envs/lcm_agent.py 文件，可以知道：
// 神经网络只输出期望的q，而kp，kd是可以自定义设置的, qd 和 tau_ff 被设置为0
void Custom::lcm_receive_Handler(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt* msg){
    (void) rbuf;
    (void) chan;


    printf("PZH: Received high-level command from Python\n");
    // TODO(PZH): We should write the high-level command to somebody.
    // TODO(PZH): We should make another buddy process the high-level command.
    //    joint_command_simple = *msg; //接收神经网络输出的关节信号

}

void Custom::lcm_high_receive_Handler(const lcm::ReceiveBuffer *rbuf, const std::string & chan, const pd_tau_targets_lcmt* msg){
    (void) rbuf;
    (void) chan;


    printf("PZH: Received high-level command from Python\n");
    // TODO(PZH): We should write the high-level command to somebody.
    // TODO(PZH): We should make another buddy process the high-level command.
    //    joint_command_simple = *msg; //接收神经网络输出的关节信号

    printf("This function should not be called now as we don't subscribe");
    sport_client.Move(0.0, 0.0, 0.1);

}





// 此处参考lcm推荐的标准格式，循环处理，接受lcm消息
void Custom::lcm_receive(){

    printf("In lcm receive, setting move...\n");
    sport_client.Move(0.0, 0.0, 0.3);

//    while(true){
//        lc.handle();
//    }
}


// -------------------------------------------------------------------------------
// 线程 3 ： unitree_sdk2 command write 线程
// 此线程作用：初始化low_cmd，经过合理的状态机后，电机将执行神经网络的输出
void Custom::InitLowCmd()
{
    //LowCmd 类型中的 head 成员 表示帧头，
    //此帧头将用于 CRC 校验。head 、levelFlag、gpio 等按例程所示设置为默认值即可。
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    /*LowCmd 类型中有 20 个 motorCmd 成员，
    每一个成员的命令用于控制 Go2 机器人上相对应的一个电机，
    但 Go2 机器人上只有 12 个电机，
    故仅有前 12 个有效，剩余的8个起保留作用。*/
    for(int i=0; i<20; i++)
    {
        /*此行命令中将 motorCmd 成员的 mode 变量设置为 0x01，
        0x01 表示将电机设置为伺服模式。
        如果用户在调试过程中发现无法控制 Go2 机器人的关节电机，
        请检查变量的值是否为0x01。*/
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}



void Custom::SetStandUp(){
    // 运行此cpp文件后，不仅是初始化通信
    // 同样会在趴下时的初始化关节角度
    // 将各个电机都设置为位置模式
//    for(int i = 0; i < 12; i++){
//        joint_command_simple.qd_des[i] = 0;
//        joint_command_simple.tau_ff[i] = 0;
//        joint_command_simple.kp[i] = 20;
//        joint_command_simple.kd[i] = 0.5;
//    }
//
//    // 趴下时的关节角度
//    joint_command_simple.q_des[0] = -0.3;
//    joint_command_simple.q_des[1] = 1.2;
//    joint_command_simple.q_des[2] = -2.721;
//    joint_command_simple.q_des[3] = 0.3;
//    joint_command_simple.q_des[4] = 1.2;
//    joint_command_simple.q_des[5] = -2.721;
//    joint_command_simple.q_des[6] = -0.3;
//    joint_command_simple.q_des[7] = 1.2;
//    joint_command_simple.q_des[8] = -2.721;
//    joint_command_simple.q_des[9] = 0.3;
//    joint_command_simple.q_des[10] = 1.2;
//    joint_command_simple.q_des[11] = -2.721;

//    std::cout<<"SET NOMINAL POSE"<<std::endl;

    // PZH: Do not use StandUp as it locks the joints. The BalanceStand will enable further movement.
    //    sport_client.SwitchGait(idle); //idle步态
    //    sport_client.StandUp(); //站立，关节锁死

    // TODO(PZH): Understand what default values should be used for Euler??
    sport_client.Euler(0.1, 0.2, 0.3); // 输入参数分别为roll, pitch, yaw角度
    sport_client.BodyHeight(1.0);      // 机身的相对高度，0对应0.33m
    sport_client.BalanceStand();       //平衡站立

//    sport_client.StandUp();
// TODO(PZH): Rename this function
    printf("PZH: Set state to StandUp\n");

}

void Custom::LowCmdWrite(){
    motiontime++;

    if(_firstRun && leg_control_lcm_data.q[0] != 0){
        for(int i = 0; i < 12; i++){
            // 程序首次运行至此的时候
            // 将当前各关节角度设置为目标角度
            joint_command_simple.q_des[i] = leg_control_lcm_data.q[i];
            // 初始化L2+B，防止damping被误触发
            key.components.Y = 0;
            key.components.A = 0;
            key.components.B = 0;
            key.components.L2 = 0;
        }
        _firstRun = false;
    }

    // 写了一段安全冗余代码
    // 当roll角超过限制，或pitch角超过限制，或遥控器按下L2+B键
    // if (  low_state.imu_state().rpy()[0] > 0.5 || low_state.imu_state().rpy()[1] > 0.5 || ((int)key.components.B==1 && (int)key.components.L2==1))
    if ( std::abs(low_state.imu_state().rpy()[0]) > 0.8 || std::abs(low_state.imu_state().rpy()[1]) > 0.8 || ((int)key.components.B==1 && (int)key.components.L2==1))
    {
        for (int i = 0; i < 12; i++){
            // 进入damping模式
            low_cmd.motor_cmd()[i].q() = 0;
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].kd() = 5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
        std::cout << "======= Switched to Damping Mode, and the thread is sleeping ========"<<std::endl;
        sleep(1.5);

        while (true)
        {

            // sleep(0.1);

            if (((int)key.components.B==1 && (int)key.components.L2==1) ) {
                // [L2+B] is pressed again
                std::cout << "======= [L2+B] is pressed again, the script is about to exit========" <<std::endl;
                exit(0);
            } else if (((int)key.components.A==1 && (int)key.components.L2==1) ){
                rsc.ServiceSwitch("sport_mode", 1);
                std::cout << "======= activate sport_mode service and exit========" <<std::endl;
                sleep(0.5);
                exit(0);
            } else{
                if (((int)key.components.Y==1 && (int)key.components.L2==1) ){
                    std::cout << "=======  Switch to Walk These Ways ========"<<std::endl;
                    std::cout<<"Communicatino is set up successfully" << std::endl;
                    std::cout<<"LCM <<<------------>>> Unitree SDK2" << std::endl;
                    std::cout<<"------------------------------------" << std::endl;
                    std::cout<<"------------------------------------" << std::endl;
                    std::cout<<"Press L2+B if any unexpected error occurs" << std::endl;
                    break;

                }else{
                    std::cout << "======= Press [L2+B] again to exit ========"<<std::endl;
                    std::cout << "======= Press [L2+Y] again to switch to WTW ========"<<std::endl;
                    std::cout << "======= Press [L2+A] again to activate sport_mode service========"<<std::endl;
                    sleep(0.01);
                }

            }

        }

    }
    else{
        for (int i = 0; i < 12; i++){
            // 在确保安全的前提下，才执行神经网络模型的输出
            low_cmd.motor_cmd()[i].q() = joint_command_simple.q_des[i];
            low_cmd.motor_cmd()[i].dq() = joint_command_simple.qd_des[i];
            low_cmd.motor_cmd()[i].kp() = joint_command_simple.kp[i];
            low_cmd.motor_cmd()[i].kd() = joint_command_simple.kd[i];
            low_cmd.motor_cmd()[i].tau() = joint_command_simple.tau_ff[i];
        }
    }

    /*此段代码中第一行首先计算了 CRC 校验码。
    最后一行代码表示调用 lowcmd_publisher的Write()函数将控制命令发送给 Go2 机器人。*/
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    lowcmd_publisher->Write(low_cmd);
}


void Custom::HighCmdWrite(){
    high_motiontime++;

    if(_firstRun && leg_control_lcm_data.q[0] != 0){
        for(int i = 0; i < 12; i++){
            // 程序首次运行至此的时候
            // 将当前各关节角度设置为目标角度
            joint_command_simple.q_des[i] = leg_control_lcm_data.q[i];
            // 初始化L2+B，防止damping被误触发
            key.components.Y = 0;
            key.components.A = 0;
            key.components.B = 0;
            key.components.L2 = 0;
        }
        _firstRun = false;
    }

    // 写了一段安全冗余代码
    // 当roll角超过限制，或pitch角超过限制，或遥控器按下L2+B键
    // if (  low_state.imu_state().rpy()[0] > 0.5 || low_state.imu_state().rpy()[1] > 0.5 || ((int)key.components.B==1 && (int)key.components.L2==1))
    if ( std::abs(low_state.imu_state().rpy()[0]) > 0.8 || std::abs(low_state.imu_state().rpy()[1]) > 0.8 || ((int)key.components.B==1 && (int)key.components.L2==1))
    {
        for (int i = 0; i < 12; i++){
            // 进入damping模式
            low_cmd.motor_cmd()[i].q() = 0;
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].kd() = 5;
            low_cmd.motor_cmd()[i].tau() = 0;
        }
        std::cout << "======= Switched to Damping Mode, and the thread is sleeping ========"<<std::endl;
        sleep(1.5);

        while (true)
        {

            // sleep(0.1);

            if (((int)key.components.B==1 && (int)key.components.L2==1) ) {
                // [L2+B] is pressed again
                std::cout << "======= [L2+B] is pressed again, the script is about to exit========" <<std::endl;
                exit(0);
            } else if (((int)key.components.A==1 && (int)key.components.L2==1) ){
                rsc.ServiceSwitch("sport_mode", 1);
                std::cout << "======= activate sport_mode service and exit========" <<std::endl;
                sleep(0.5);
                exit(0);
            } else{
                if (((int)key.components.Y==1 && (int)key.components.L2==1) ){
                    std::cout << "=======  Switch to Walk These Ways ========"<<std::endl;
                    std::cout<<"Communicatino is set up successfully" << std::endl;
                    std::cout<<"LCM <<<------------>>> Unitree SDK2" << std::endl;
                    std::cout<<"------------------------------------" << std::endl;
                    std::cout<<"------------------------------------" << std::endl;
                    std::cout<<"Press L2+B if any unexpected error occurs" << std::endl;
                    break;

                }else{
                    std::cout << "======= Press [L2+B] again to exit ========"<<std::endl;
                    std::cout << "======= Press [L2+Y] again to switch to WTW ========"<<std::endl;
                    std::cout << "======= Press [L2+A] again to activate sport_mode service========"<<std::endl;
                    sleep(0.01);
                }

            }

        }

    }
    else{
        for (int i = 0; i < 12; i++){
            // 在确保安全的前提下，才执行神经网络模型的输出
            low_cmd.motor_cmd()[i].q() = joint_command_simple.q_des[i];
            low_cmd.motor_cmd()[i].dq() = joint_command_simple.qd_des[i];
            low_cmd.motor_cmd()[i].kp() = joint_command_simple.kp[i];
            low_cmd.motor_cmd()[i].kd() = joint_command_simple.kd[i];
            low_cmd.motor_cmd()[i].tau() = joint_command_simple.tau_ff[i];
        }
    }

    /*此段代码中第一行首先计算了 CRC 校验码。
    最后一行代码表示调用 lowcmd_publisher的Write()函数将控制命令发送给 Go2 机器人。*/
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
    lowcmd_publisher->Write(low_cmd);
}


//
// 与循环工作的线程相关的函数定义已完结
//----------------------------------------------------------------------



void Custom::Init(){
    _firstRun = true;



    // PZH: Initialize sport client
    sport_client.SetTimeout(10.0f);
    sport_client.Init();
    printf("PZH: SportClient is set up successfully.\n");

    // PZH: Let the dog stand up
//    InitLowCmd();
    SetStandUp();
    printf("PZH: Should already set to stand up\n");


    // PZH: Setup LCM listener, listening commands from Python.
    // 这里决定了调用lc.handle()的时候，订阅什么消息，进行什么操作
    // 订阅什么消息："pd_plustau_targets"
    // 进行什么操作： lcm_receive_Handler
    // PZH: TODO: so we should subscribe to some high-level commend (from python probably) and call relevent
    //        processor.
    //    lc.subscribe("pd_plustau_targets", &Custom::lcm_receive_Handler, this);
// PZH: TODO: We don't use subscription now, just overwrite the lcm_receive to publish highlevel cmd.
// TODO(PZH): We should use subscription when receiving signal from python.
//        lc.subscribe("pd_plustau_targets_high", &Custom::lcm_high_receive_Handler, this);


    // PZH: Setup LCM publisher, publish the info in robot to Python.
    /*create low_cmd publisher*/
//    lowcmd_publisher.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
//    lowcmd_publisher->InitChannel();
    /*create low_state dds subscriber*/
//    lowstate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
//    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);
    /*create joystick dds subscriber*/
//    joystick_suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
//    joystick_suber->InitChannel(std::bind(&Custom::JoystickHandler, this, std::placeholders::_1), 1);

// TODO(PZH): Seems like the high level subscriber is not used in lcm_receive ??
    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
}


void Custom::Loop(){
    // 新增线程可以实现loop function的功能

    // intervalMicrosec : 1微秒 = 0.000001秒
    // 当dt=0.002s
    // ntervalMicrosec = 2000us



    /*lcm send thread*/
    LcmSendThreadPtr = unitree::common::CreateRecurrentThreadEx("lcm_send_thread", UT_CPU_ID_NONE, dt*1e6, &Custom::lcm_send, this);
    /*lcm receive thread*/

    // TODO(PZH): In correct dt
    float robot_dt = 0.005;
    LcmRecevThreadPtr = unitree::common::CreateRecurrentThreadEx("lcm_receive_thread", UT_CPU_ID_NONE, robot_dt*1e6, &Custom::lcm_receive, this);
    /*low command write thread*/
//    lowCmdWriteThreadPtr = unitree::common::CreateRecurrentThreadEx("dds_write_thread", UT_CPU_ID_NONE, dt*1e6, &Custom::LowCmdWrite, this);
//    unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::RobotControl, &custom));

//    float robot_dt = 0.005;      // 控制步长0.001~0.01
//    lowCmdWriteThreadPtr = unitree::common::CreateRecurrentThreadEx("dds_high_write_thread", UT_CPU_ID_NONE, robot_dt*1e6, &Custom::HighCmdWrite, this);


}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

//    std::cout << "Communication level is set to LOW-level." << std::endl
//              << "WARNING: Make sure the robot is hung up." << std::endl
//              << "Caution: The scripts is about to shutdown Unitree sport_mode Service." << std::endl
//              << "Press Enter to continue..." << std::endl;

    printf("PZH: We will set communication level to HIGH-level. Press Enter to continue...\n");
    std::cin.ignore();

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]); // 传入本机的网卡地址（PC or Jetson Orin）

    Custom custom;

    custom.InitRobotStateClient();
    printf("PZH: Robot State Client is set up successfully.\n");

    if(custom.queryServiceStatus("sport_mode"))
    {
        std::cout <<"sport_mode is already activated now" << std::endl
                  <<"next step is setting up communication" << std::endl
                  << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
    } else{
        std::cout<<"Trying to ACTIVATE the service: " << "sport_mode" << std::endl;
        custom.activateService("sport_mode",1);
        sleep(0.5);
//        if(custom.queryServiceStatus("sport_mode")){
//            std::cout<<"Trying to ACTIVATE the service: " << "sport_mode" << std::endl;
//        }
    }


    custom.queryServiceStatus("sport_mode");

    printf("PZH: Start initializing the communication ...");
    custom.Init();

    std::cout<<"Communication is set up successfully" << std::endl;
    std::cout<<"LCM <<<------------>>> Unitree SDK2" << std::endl;
    std::cout<<"------------------------------------" << std::endl;
    std::cout<<"------------------------------------" << std::endl;
    std::cout<<"Press L2+B if any unexpected error occurs" << std::endl;

    custom.Loop();
    printf("PZH: Communication threads are running ...");

    while (true)
    {
        sleep(10);
    }

    return 0;
}