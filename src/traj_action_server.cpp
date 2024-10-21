//utility
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "pkg_interfaces/action/traj.hpp"
#include "pkg_main/quintic.hpp"
#include <iostream>

class JointTrajActionServer : public rclcpp::Node
{
  using JointTraj = pkg_interfaces::action::Traj;
  using GoalHandleJointTraj = rclcpp_action::ServerGoalHandle<JointTraj>;
  
  protected:
    rclcpp_action::Server<JointTraj>::SharedPtr traj_action_server_;  //action server per la action traj_action_server
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_; //publisher per il topic /cmd/joint_position

    std::vector<std::string> joint_names_ = {"panda_joint1",
                                           "panda_joint2",
                                           "panda_joint3",
                                           "panda_joint4",
                                           "panda_joint5",
                                           "panda_joint6",
                                           "panda_joint7"};

  public:
  JointTrajActionServer(const rclcpp::NodeOptions opt = rclcpp::NodeOptions()) : Node("traj_action_server", opt)
  {
    using namespace std::placeholders;
    
    joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd/joint_position", 1);

    this->traj_action_server_ = rclcpp_action::create_server<JointTraj>(
        this, "traj_action_server", std::bind(&JointTrajActionServer::handle_goal, this, _1, _2),
        std::bind(&JointTrajActionServer::handle_cancel, this, _1),
        std::bind(&JointTrajActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Joint Traj Action Server avviato");
  }


 //distruttore
 ~JointTrajActionServer() = default;


 protected:

 rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const JointTraj::Goal> goal)
  {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Goal request arrivato with\n durata: " << builtin_interfaces::msg::to_yaml(goal->time));
                                                                 
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleJointTraj> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Ricevuta la richiesta di cancellazione goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleJointTraj> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&JointTrajActionServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleJointTraj> goal_handle)
  {
    using namespace std::chrono_literals;

    auto goal = goal_handle->get_goal();

    //Il seguente blocco serve per leggere la prima configurazione q0.
    // INIZIO BLOCCO PER LETTURA GIUNTI
    sensor_msgs::msg::JointState q0;
    if (!rclcpp::wait_for_message<sensor_msgs::msg::JointState>(q0, shared_from_this(), "joint_states", 10s))
    {
      RCLCPP_ERROR(this->get_logger(), "Fail nell'ottenere lo stato dei giunti iniziale q0");
      auto result = std::make_shared<JointTraj::Result>();
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    double qf[]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    for(int j=0;j<(int)joint_names_.size(); j++)
    {
       qf[j]=goal->qf[j]+q0.position[j];
    }


   
    // FINE BLOCCO PER LETTURA GIUNTI

    RCLCPP_INFO(this->get_logger(), "q0 LETTA");

    // stampa per visualizzarla
    //RCLCPP_INFO_STREAM(this->get_logger(), "\nqi is:\n" << sensor_msgs::msg::to_yaml(q0));
    
    rclcpp::Time t0 = this->now();
    rclcpp::Duration t(0, 0);  // inizializzo t=0;

    // La traiettoria sarà pubblicata con una certa frequenza (1 KHz in questo caso)
    double frequency=1000.0; 
    rclcpp::Rate loop_rate(frequency);

    // Estraggo la duration in secondi
    double traj_duration = rclcpp::Duration(goal->time).seconds();

    /*
      Ciclo di "generazione traiettoria"
      cicla finchè ros è ok e non ho finito la traiettoria
      t <= goal->duration significa t<tf.
    */

    //double delta[] {0, 0, 0, 0, 0.17, 0.17, 0.17};

    double velocity[]={0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    sensor_msgs::msg::JointState cmd;
    cmd.position=q0.position;

    while (rclcpp::ok() && t <= goal->time)
    {
      // calcolo il nuovo t
        t = this->now() - t0;

        // check preemprtion dell'azione
        if (goal_handle->is_canceling())
        {
          auto result = std::make_shared<JointTraj::Result>();
          result->success= false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Joint traj Goal cancellato");
          return;
        }

        // riempio il messaggio di comando
       

        // cmd.position è un std::vector. Di default ha size=0. Devo fare un resize
        cmd.position.resize(joint_names_.size());
        cmd.velocity.resize(joint_names_.size());

        // riempio il vettore usando la funzione quintic che ho creato nella libreria
        // qui ho supposto che q0 è già ordinato correttamente
        
        for (int i = 0; i <(int)joint_names_.size(); i++)
        {
          //cmd.position[i] = quintic(t.seconds(), q0.position[i], goal->qf[i], traj_duration); 
          //cmd.velocity[i] = quintic(t.seconds(), q0.position[i], goal->qf[i], traj_duration); 
          velocity[i]=quintic(t.seconds(), q0.position[i], qf[i], traj_duration); 
          cmd.velocity[i] =velocity[i];
          cmd.position[i]=cmd.position[i]+(1/frequency)*velocity[i];
          //std::cout<< "CMD.POSITION: "<<cmd.position[5]<<std::endl;
        }

        std::cout<< "CMD.POSITION: "<<cmd.position[5]<<std::endl;
        // è importante riempire il vettore dei nomi dei giunti.
        cmd.name = joint_names_;

        // riempio lo stamp, utile in caso di plot
        cmd.header.stamp = this->now();

        // ho definito come feedback il 'tempo che manca alla fine della traiettoria'
        // ovviamente non è l'unica scelta.
        auto feedbk_msg = std::make_shared<JointTraj::Feedback>();
        feedbk_msg->q_act = cmd.position;

        // pubblico il feedback
        goal_handle->publish_feedback(feedbk_msg);

        // publico il comando in giunti
        joint_cmd_pub_->publish(cmd);


        // sleep sul rate
        loop_rate.sleep();
    }
    
    // se sono arrivato fin qui, non ci sono stati errori o preemption e la traiettoria è finita
    if (rclcpp::ok())
    {
      auto result = std::make_shared<JointTraj::Result>();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Joint Traj Goal succeeded");
    }

  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTrajActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
