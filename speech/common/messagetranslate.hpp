#ifndef MESSAGETRANSLATE_H
#define MESSAGETRANSLATE_H

#include <unistd.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace message {

template<
    typename PublishMessage = std_msgs::msg::String,
    typename SubscribeMessage = std_msgs::msg::String
    >
class MessageTranslate : public rclcpp::Node
{
public:
  MessageTranslate(const std::string &nodeName)
    : Node(nodeName)
  {
  }

  void CreatePublisher(const std::string &topic)
  {
    m_pPublisher = this->create_publisher<PublishMessage>(topic);
  }

  template<typename SubscriberCallback>
  void CreateSubscriber(
      const std::string &topic,
      SubscriberCallback &&callback
      )
  {
    m_pSubscription = this->create_subscription<SubscribeMessage>(
          topic, callback);
  }

  bool CreatePublisherWithTopic(const std::string &topic)
  {
    auto it = m_pPublisherWithTopic.find(topic);
    if (it != m_pPublisherWithTopic.end()) return false;

    m_pPublisherWithTopic[topic] = this->create_publisher<PublishMessage>(topic);
    return true;
  }

  template<typename SubscriberCallback>
  bool CreateSubscriberWithTopic(
      const std::string &topic,
      SubscriberCallback &&callback
      )
  {
    auto it = m_pSubscriptionWithTopic.find(topic);
    if (it != m_pSubscriptionWithTopic.end()) return false;

    m_pSubscriptionWithTopic[topic] = this->create_subscription<SubscribeMessage>(
          topic, callback);
    return true;
  }

  void PublishMsg(const PublishMessage &message)
  {
    m_pPublisher->publish(message);
  }

  void PublishMsgWithTopic(const std::string &topic, const PublishMessage &message)
  {
    auto it = m_pPublisherWithTopic.find(topic);
    if (it == m_pPublisherWithTopic.end()) return;
    it->second->publish(message);
  }

  void wait_for_topics(const std::vector<std::string> &topics)
  {
    while (true) {
      std::map< std::string, std::vector< std::string > > &&topic_names_types = this->get_topic_names_and_types();
      size_t number = 0;
      for (const auto &topic : topics) {
        std::string topic_full = "/" + topic;
        auto it = topic_names_types.find(topic_full);
        if (it != topic_names_types.end()) ++ number;
      }
      if (number == topics.size()) break;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  template<typename ServiceT>
  typename rclcpp::Client<ServiceT>::SharedPtr CreateServiceClicent(
      const std::string &service,
      const rmw_qos_profile_t &qos = rmw_qos_profile_services_default,
      rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
  {
    typename rclcpp::Client<ServiceT>::SharedPtr client = this->create_client<ServiceT>(service, qos, group);
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            return nullptr;
        }
    }
    return client;
  }

  template<typename ServiceT,
           typename ServiceT::Request,
           typename ServiceT::response>
  std::shared_ptr<typename ServiceT::response> DoRequestResponse(
      typename rclcpp::Client<ServiceT>::SharedPtr client,
      const std::shared_ptr<typename ServiceT::Request> request,
      int ms = 1000)
  {
    auto self = shared_from_this();
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(self, result_future, std::chrono::milliseconds(ms)) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return nullptr;
    }
    auto result = result_future.get();
    return result;
  }

  template<typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr CreateServiceServer(
      const std::string &service,
      CallbackT &&callback,
      const rmw_qos_profile_t &qos_profile = rmw_qos_profile_services_default,
      rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
  {
    return this->create_service<ServiceT>(
              service, callback, qos_profile, group);
  }

  template<typename ActionT>
  typename rclcpp_action::Server<ActionT>::SharedPtr CreateActionServer(
      const std::string & name,
      typename rclcpp_action::Server<ActionT>::GoalCallback handle_goal,
      typename rclcpp_action::Server<ActionT>::CancelCallback handle_cancel,
      typename rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted,
      const rcl_action_server_options_t & options = rcl_action_server_get_default_options(),
      rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
  {
    auto self = shared_from_this();
    auto action_server = rclcpp_action::create_server<ActionT>(
          self,
          name,
          handle_goal,
          handle_cancel,
          handle_accepted,
          options,
          group);
    return action_server;
  }

  template<typename ActionT>
  typename rclcpp_action::Client<ActionT>::SharedPtr
  CreateActionClient(
      const std::string & name,
      int seconds = 0,
      rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
  {
    auto self = shared_from_this();
    auto action_client = rclcpp_action::create_client<ActionT>(self, name, group);

    if (seconds > 0 && !action_client->wait_for_action_server(std::chrono::seconds(seconds))) {
        return nullptr;
    }
    return action_client;
  }

  template<typename ActionT,
           typename CallbackT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::Result DoAction(
      typename rclcpp_action::Client<ActionT>::SharedPtr action_client,
      const typename ActionT::Goal &goal,
      CallbackT &&callback)
  {
    auto self = shared_from_this();
    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = action_client->async_send_goal(goal, callback);

    if (rclcpp::spin_until_future_complete(self, goal_handle_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return;
    }

    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle = goal_handle_future.get();
    if (!goal_handle) {
        return ;
    }

    // Wait for the server to be done with the goal
    auto result_future = goal_handle->async_result();
    if (rclcpp::spin_until_future_complete(self, result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        return;
    }

    typename rclcpp_action::ClientGoalHandle<ActionT>::Result result = result_future.get();
    return result;
  }

  void Spin()
  {
    std::thread *pThread = new std::thread(
          std::mem_fn(&MessageTranslate::OnThreadSpin), this);
    m_pThreadSpin.reset(pThread);
  }

  void Stop()
  {
    rclcpp::shutdown();
    m_pThreadSpin->join();
  }

  static std::string get_local_ip(const std::string &prefix)
  {
    char *ip = nullptr;
    int fd, intrface;
    struct ifreq buf[16];
    struct ifconf ifc;
    std::string strIP;

    while (true)
    {
      if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) >= 0)
      {
        ifc.ifc_len = sizeof(buf);
        ifc.ifc_buf = (caddr_t)buf;
        if (!ioctl(fd, SIOCGIFCONF, (char *)&ifc))
        {
          intrface = ifc.ifc_len / sizeof(struct ifreq);
          while (intrface-- > 0)
          {
            if (!(ioctl (fd, SIOCGIFADDR, (char *) &buf[intrface])))
            {
              ip=(inet_ntoa(((struct sockaddr_in*)(&buf[intrface].ifr_addr))->sin_addr));
              strIP = ip;
              std::size_t pos = strIP.find(prefix);
              if (pos == 0) break;
              strIP.clear();
            }
          }
        }
        close (fd);
      }
      if (!strIP.empty()) break;
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    return strIP;
  }

protected:
  void OnThreadSpin()
  {
    rclcpp::spin(shared_from_this());
  }

private:
  std::shared_ptr<rclcpp::Subscription<SubscribeMessage>> m_pSubscription;
  std::shared_ptr<rclcpp::Publisher<PublishMessage>> m_pPublisher;
  std::map<std::string, std::shared_ptr<rclcpp::Subscription<SubscribeMessage>>> m_pSubscriptionWithTopic;
  std::map<std::string, std::shared_ptr<rclcpp::Publisher<PublishMessage>>> m_pPublisherWithTopic;

  std::unique_ptr<std::thread> m_pThreadSpin;
};

}

#endif // MESSAGETRANSLATE_H
