#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <toml.hpp>
#include <utility>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
#include "task_msg/msg/float64_pair_multi_array.hpp"
#include "task_msg/msg/float64_multi_array.hpp"

#include "task_msg/msg/float64_tri_multi_array.hpp"

#include "task_msg/srv/point2d_move.hpp"
#include "task_msg/srv/nav_find.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace behaviourDecision{

        class behaviour_message
        {
            public:
                double enemy_blood[1000];
                double enemy_pos[1000][2];  //要是早點想到就用eigen了
                int enemy_num;

                float self_blood;
                double self_v[2];
                double self_w;
                double self_position[2];
                
        };
        
        class publish_ans_class
        {
            public:
                float end_point_global_planner[2];
                float decision_shooting_angle;
                int decision_shooting;
                publish_ans_class(int flag):decision_shooting(flag){}
        };
        behaviour_message message;
        publish_ans_class ans(1);

        const auto map_information = toml::parse(ROOT "src/config.toml");
        const auto stone_position= toml::find<std::vector<std::vector<double>>>(map_information,"STONE_OF_QUEUE");
        static double stone_pos[10][2];
        static int sum_stone=0;
        const double eps=0.00001;

    class BehaviourTree:public rclcpp::Node{
        public:
        rclcpp::Publisher<task_msg::msg::Float64MultiArray>::SharedPtr  publisher_global_plan_pos;
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr  publisher_shooter;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  publisher_avail_angle;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_self_blood_point;
        rclcpp::Subscription<task_msg::msg::Float64TriMultiArray>::SharedPtr subscriber_enemy_detect_pos;
        rclcpp::Subscription<task_msg::msg::Float64MultiArray>::SharedPtr subscriber_meter_velocity;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_meter_angular_velocity;
        rclcpp::Subscription<task_msg::msg::Float64MultiArray>::SharedPtr subscriber_meter_position;


        
        
        
        explicit BehaviourTree(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
            : Node("behaviour_tree", options){
                using namespace std::placeholders;

                timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&BehaviourTree::behaviour_publish, this));
                publisher_global_plan_pos = this->create_publisher<task_msg::msg::Float64MultiArray>("end_point_global_planner",10);
                publisher_avail_angle = this->create_publisher<std_msgs::msg::Float64>("decision_shooting_angle",10);
                publisher_shooter = this->create_publisher<std_msgs::msg::Int64>("decision_shooting_able",10);

                subscriber_self_blood_point = this->create_subscription<std_msgs::msg::Float64>("self_health_point", 10, std::bind(&BehaviourTree::receive_self_blood_point, this, _1));
                subscriber_enemy_detect_pos = this->create_subscription<task_msg::msg::Float64TriMultiArray>("enemy_detect_pos", 10, std::bind(&BehaviourTree::receive_enemy_detect_pos, this, _1));
                subscriber_meter_velocity = this->create_subscription<task_msg::msg::Float64MultiArray>("meter_velocity", 10, std::bind(&BehaviourTree::receive_meter_velocity, this, _1));
                subscriber_meter_angular_velocity = this->create_subscription<std_msgs::msg::Float64>("meter_angular_velocity", 10, std::bind(&BehaviourTree::receive_meter_angular_velocity, this, _1));
                subscriber_meter_position = this->create_subscription<task_msg::msg::Float64MultiArray>("meter_position", 10, std::bind(&BehaviourTree::receive_meter_position, this, _1));

                for(auto i:stone_position)
                {
                    stone_pos[sum_stone][0]=i[0];
                    stone_pos[sum_stone][1]=i[1];
                    sum_stone++;
                }
            }
        


        void behaviour_publish(){
            //const double pi = 3.14159265358979323;
            //static double CNT = 0;
            //while(CNT > 2*pi) CNT -= 2*pi;
            //RCLCPP_INFO(this->get_logger(), "現在你的w是%lf\n", message.self_w);
            std::vector<double> vec;
            double angle = ans.decision_shooting_angle-0.01*message.self_w;
            std_msgs::msg::Float64 pub_angle;
            pub_angle.data = angle;
            vec.push_back(ans.end_point_global_planner[0]);
            vec.push_back(ans.end_point_global_planner[1]);
            task_msg::msg::Float64MultiArray pub_msg;
            pub_msg.data = vec;
            publisher_global_plan_pos->publish(pub_msg);
            publisher_avail_angle->publish(pub_angle);
            std_msgs::msg::Int64 able_msg;
            able_msg.data = ans.decision_shooting;
            publisher_shooter->publish(able_msg);
            return;
        }

        double calculate_distance(double a,double b,double c,double d)
        {
            return sqrt(pow(a-b,2)+pow(c-d,2));
        }
        //ing......
        void behaviour_decision()
        {
            const double pi = 3.14159265358979323;
            

            //找尋一個目標,因爲player優秀的擊殺能力,其實哪個目標並不重要
            int enemy_min_blood=0,pos_enemy_minblood=0;
            for(int i=0;i<message.enemy_num;i++)
            {
                if(message.enemy_blood[i]<enemy_min_blood)
                {
                    enemy_min_blood=message.enemy_blood[i];
                    pos_enemy_minblood=i;
                }
            }
            //是否發射子彈
            //double diatance=pow(pow((message.enemy_pos[pos_enemy_minblood][1]-message.self_position[1]),2)+pow((message.enemy_pos[pos_enemy_minblood][0]-message.self_position[0]),2),0.5);

            
            //RCLCPP_INFO(this->get_logger(), "the number%d\n", sum_stone);
            //儘量實現不射到礦石
            /*int flag_tongce=0,flag_gongxian=0;
            if(message.enemy_num)
            {
                ans.decision_shooting=1;
                for(int i=0;i<sum_stone;i++)
                {
                    if((stone_pos[i][0]-message.self_position[0])*(stone_pos[i][0]-message.enemy_pos[pos_enemy_minblood][0])>=0)
                    flag_tongce=1;
                    else if(abs((stone_pos[i][0]-message.self_position[0])/(stone_pos[i][1]-message.self_position[1]))-abs((stone_pos[i][0]-message.enemy_pos[pos_enemy_minblood][0])/(stone_pos[i][1]-message.enemy_pos[pos_enemy_minblood][1]))<eps)
                    flag_gongxian=1;

                    if(flag_tongce==0 && flag_gongxian==1)
                    {
                        ans.decision_shooting=0;
                        break;
                    }
                    else
                    ans.decision_shooting=1;
                    flag_tongce=0,flag_gongxian=0;
                }
            }
            else
            ans.decision_shooting=0;*/
            if(message.enemy_num)
            ans.decision_shooting=1;
            else
            ans.decision_shooting=0;

            //position_method 1.1
            static int flag_point;
            if(message.self_blood>10)//0.4*message.enemy_blood[pos_enemy_minblood]
            {
                if(message.enemy_num)
                    {
                    ans.end_point_global_planner[0]=message.enemy_pos[pos_enemy_minblood][0];
                    ans.end_point_global_planner[1]=message.enemy_pos[pos_enemy_minblood][1];
                    }
                else
                {   
                    //RCLCPP_INFO(this->get_logger(),"the distance:%lf\n",calculate_distance(ans.end_point_global_planner[0],message.self_position[0],ans.end_point_global_planner[1],message.self_position[1]));
                    //RCLCPP_INFO(this->get_logger(), "現在你的位置是(%lf,%lf) 現在stone的位置是(%lf,%lf)\n", message.self_position[0],message.self_position[1],ans.end_point_global_planner[0],ans.end_point_global_planner[1]);
                    if(calculate_distance(ans.end_point_global_planner[0],message.self_position[0],ans.end_point_global_planner[1],message.self_position[1])<0.8)// || (message.self_v[0]<=0.1 && message.self_v[1]<=0.1)
                        flag_point=1-flag_point;
                    if(flag_point)
                    {
                    ans.end_point_global_planner[0]=stone_pos[1][0]+3.5;
                    ans.end_point_global_planner[1]=stone_pos[1][1]-1;
                    }
                    else
                    {
                    ans.end_point_global_planner[0]=stone_pos[0][0]-1.3;
                    ans.end_point_global_planner[1]=stone_pos[0][1]+1.5;
                    }
                }
            }
            else
            {
                ans.end_point_global_planner[0]=1.5;
                ans.end_point_global_planner[1]=6.5;
            }
            //

            //ans.decision_shooting_angle=atan();
            //未解決問題:w對設子彈的影響!!!!!!!!!!!!
            double tangent=(message.enemy_pos[pos_enemy_minblood][1]-message.self_position[1])/(message.enemy_pos[pos_enemy_minblood][0]-message.self_position[0]);
            if(message.enemy_pos[pos_enemy_minblood][0]>message.self_position[0])
            ans.decision_shooting_angle=atan(tangent);
            else if(atan(tangent)<0)
            ans.decision_shooting_angle=pi+atan(tangent);
            else
            ans.decision_shooting_angle=(atan(tangent)+pi*(-1));
            behaviour_publish();
            return;
        }




        //finished! data_receive_function
        void receive_self_blood_point(const std_msgs::msg::Float64::SharedPtr blood)
        {
            message.self_blood=blood->data;
            behaviour_decision();
            //RCLCPP_INFO(this->get_logger(), "現在你的血量是%f\n", blood->data);

        }
        void receive_enemy_detect_pos(const task_msg::msg::Float64TriMultiArray::SharedPtr pos)
        {
            message.enemy_num=pos->data.size();
            for(int i=0;i<(int)pos->data.size();i++)
            {
                message.enemy_pos[i][0]=pos->data[i].x;
                message.enemy_pos[i][1]=pos->data[i].y;
                message.enemy_blood[i]=pos->data[i].z;
            }
            behaviour_decision();
            
            //for(int i=0;i<(int)pos->data.size();i++)
            //RCLCPP_INFO(this->get_logger(), "現在%d號敵人的位置是(%f,%f),ta的血量是%f\n", i+1,pos->data[i].x,pos->data[i].y,pos->data[i].z);
        }
        void receive_meter_velocity(const task_msg::msg::Float64MultiArray::SharedPtr v)
        {
            message.self_v[0]=v->data[0];
            message.self_v[1]=v->data[1];
            behaviour_decision();
            //RCLCPP_INFO(this->get_logger(), "現在你的速度是%lf\n", pow((v->data[0])*(v->data[0])+(v->data[1])*(v->data[1]),0.5));
        }
        void receive_meter_angular_velocity(const std_msgs::msg::Float64::SharedPtr w)
        {
            //RCLCPP_INFO(this->get_logger(), "現在你的角速度是%lf\n", w->data);
            message.self_w=w->data;
            
            behaviour_decision();
            
        }
        void receive_meter_position(const task_msg::msg::Float64MultiArray::SharedPtr pos)
        {
            message.self_position[0]=pos->data[0];
            message.self_position[1]=pos->data[1];
            behaviour_decision();
            //RCLCPP_INFO(this->get_logger(), "現在你的位置是(%lf,%lf)\n", pos->data[0],pos->data[1]);
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(behaviourDecision::BehaviourTree)