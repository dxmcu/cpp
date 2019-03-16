#ifndef MESSAGETRANSLATE_H
#define MESSAGETRANSLATE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/string.hpp"

namespace message {

template<
        typename PublishMessage = std_msgs::msg::String,
        typename SubscribeMessage = std_msgs::msg::String,
        typename ServerMessage = std_msgs::msg::String,
        typename ClientMessage = std_msgs::msg::String
        >
class MessageTranslate : public rclcpp::Node
{
public:
    MessageTranslate(const std::string &nodeName)
        : Node(nodeName)
    {
    }

    void PublishText(const std::string &topic, const PublishMessage &message)
    {
        // 必须加锁，不然程序会退出
        //std::unique_lock<std::mutex> lock(m_mutexPublish);
        std::shared_ptr<rclcpp::Publisher<PublishMessage>> publisher =
                this->create_publisher<PublishMessage>(topic);
        publisher->publish(message);
    }

    void CreatePublisher(const std::string &topic)
    {
        int index = m_pPublisher.size();
        m_pPublisher[index] = this->create_publisher<PublishMessage>(topic);
    }

    template<typename SubscriberCallback>
    void CreateSubscriber(
            const std::string &topic,
            SubscriberCallback &&callback
            )
    {
        int index = m_pSubscription.size();
        m_pSubscription[index] = this->create_subscription<SubscribeMessage>(
                    topic, callback);
    }

    void CreateServiceClicent(
      const std::string &service,
      const rmw_qos_profile_t &qos = rmw_qos_profile_services_default,
      rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
    {
        m_pClient = this->create_client<ClientMessage>(service, qos, group);

        m_bWaitForService = false;
        std::thread *pThread = new std::thread(
                    std::mem_fn(&MessageTranslate::OnThreadWaitForService), this);
        m_pThreadWaitForService.reset(pThread);
    }

    template<typename ServerCallback>
    void CreateServiceServer(
            const std::string &service,
            ServerCallback &&callback,
            const rmw_qos_profile_t &qos = rmw_qos_profile_services_default,
            rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
    {
        m_pServer = this->create_service<ServerMessage>(
                    service, callback, qos, group);
    }

    void Spin()
    {
        std::thread *pThread = new std::thread(
                    std::mem_fn(&MessageTranslate::OnThreadSpin), this);
        m_pThreadSpin.reset(pThread);
    }

    void PublishMsg(std::size_t i, const PublishMessage &message)
    {
        if (i >= m_pPublisher.size()) return;
        m_pPublisher[i]->publish(message);
    }

    void Stop()
    {
        rclcpp::shutdown();
        m_pThreadSpin->join();
    }

protected:
    void OnThreadSpin()
    {
        rclcpp::spin(shared_from_this());
    }

    void OnThreadWaitForService()
    {
        while (!m_pClient->wait_for_service(std::chrono::seconds(3)))
        {
            if (!rclcpp::ok()) return;
        }
        m_bWaitForService = true;
    }

private:
    std::map<int, std::shared_ptr<rclcpp::Subscription<SubscribeMessage>>> m_pSubscription;
    std::map<int, std::shared_ptr<rclcpp::Publisher<PublishMessage>>> m_pPublisher;
    std::mutex m_mutexPublish;

    std::shared_ptr<rclcpp::Service<ServerMessage>> m_pServer;
    std::shared_ptr<rclcpp::Client<ClientMessage>> m_pClient;

    std::unique_ptr<std::thread> m_pThreadSpin;
    std::unique_ptr<std::thread> m_pThreadWaitForService;
    std::atomic_bool m_bWaitForService;
};

}

#endif // MESSAGETRANSLATE_H
