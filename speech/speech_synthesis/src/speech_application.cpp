#include "speech_synthesis/speech_application.h"

SpeechApplication::SpeechApplication()
{
  m_pSpeechSynthesis.reset(new SpeechSynthesis);
}

bool SpeechApplication::start(const std::string &node_name, const std::string &topic_subscribe)
{
  if (!m_pSpeechSynthesis->login()) return false;

  m_pTranslate = std::make_shared<message::MessageTranslate<
          std_msgs::msg::String,
          std_msgs::msg::String>>(node_name);

  auto fun = std::bind(&SpeechApplication::on_subscribe_text, this,
                       std::placeholders::_1);
  m_pTranslate->CreateSubscriber(topic_subscribe, fun);
  m_pTranslate->Spin();

  return true;
}

void SpeechApplication::stop()
{
    m_pTranslate->Stop();
    m_pSpeechSynthesis->logout();
}

void SpeechApplication::on_subscribe_text(std_msgs::msg::String::UniquePtr message)
{
  std::string text = message->data;
  m_pSpeechSynthesis->post_text(text);
}
