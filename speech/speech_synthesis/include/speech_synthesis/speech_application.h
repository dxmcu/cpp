#ifndef SPEECH_APPLICATION_H
#define SPEECH_APPLICATION_H

#include "messagetranslate.hpp"
#include "speech_synthesis.h"

class SpeechApplication
{
public:
  SpeechApplication();

  bool start(const std::string &, const std::string &);
  void stop();

protected:
  void on_subscribe_text(std_msgs::msg::String::UniquePtr);

private:
  std::unique_ptr<SpeechSynthesis> m_pSpeechSynthesis;
  std::shared_ptr<message::MessageTranslate<>> m_pTranslate;
};

#endif // SPEECH_APPLICATION_H
