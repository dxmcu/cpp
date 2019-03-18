#include <jsoncpp/json/json.h>
#include "speech_recognition/application.h"

Application::Application()
{

}

Application::~Application()
{

}

void Application::Start(const std::string &nodeName, const std::string &publisher,
                        const std::string &subscriber)
{
    SpeechRecognition *recognition = new SpeechRecognition(*this);
    m_pRecognizor.reset(recognition);
    m_pRecognizor->Start();

    TextToSpeech *speech = new TextToSpeech(*this);
    m_pSpeech.reset(speech);
    m_pSpeech->Start();

    m_pTranslate = std::make_shared<message::MessageTranslate<
            std_msgs::msg::String,
            std_msgs::msg::String>>(nodeName);
    m_pTranslate->CreatePublisher(publisher);

    auto fun = std::bind(&Application::OnSubsribe, this,
                         std::placeholders::_1);
    m_pTranslate->CreateSubscriber(subscriber, fun);
    m_pTranslate->Spin();
}

void Application::Stop()
{
    m_pRecognizor->Stop();
    m_pSpeech->Stop();
    m_pTranslate->Stop();
}

void Application::OnSubsribe(std_msgs::msg::String::UniquePtr message)
{
    std::string text = message->data;
    std::cout << "#### " << text << std::endl;

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(text, root)) return;

    Json::Value sender = root["sender"];
    Json::Value receiver = root["receiver"];
    Json::Value eventName = root["eventName"];
    Json::Value content = root["content"];
    std::string strSender = sender.asString();
    std::string strReceiver = receiver.asString();
    std::string strEventName = eventName.asString();

    if (strEventName == "TextToSpeech")
    {
        std::string strText = content.asString();
        std::cout << "### text: " << strText << std::endl;
        std::shared_ptr<SpeechMsg> msg = std::make_shared<SpeechMsg>();
        msg->m_strSender = strSender;
        msg->m_strText = strText;
        m_pSpeech->PostTextMsg(msg);
    }
}

void Application::PublishMsg(std::size_t index, const std::shared_ptr<SpeechMsg> msg)
{
    Json::Value root;
    Json::Value content;
    root["sender"] = m_pTranslate->get_name();
    root["receiver"] = msg->m_strSender;
    root["eventName"] = (index == Publish_Recognition ? "SpeechRecognize" : "TextToSpeech");
    root["content"] = msg->m_strText;

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::string strMsg = Json::writeString(builder, root);

    auto message = std_msgs::msg::String();
    message.data = strMsg;
    m_pTranslate->PublishMsg(0, message);
}

