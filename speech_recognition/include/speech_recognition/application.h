#ifndef APPLICATION_H
#define APPLICATION_H

#include "speech_recognition/speech_recognition.h"
#include "speech_recognition/text_to_speech.h"
#include "../common/messagetranslate.hpp"

class Application
{
public:
    enum
    {
        Publish_Recognition,
        Publish_Speech
    };

public:
    Application();
    ~Application();

    void Start(const std::string &, const std::string &, const std::string &);
    void Stop();

    void PublishMsg(int, const std::shared_ptr<SpeechMsg>);

protected:
    void OnSubsribe(std_msgs::msg::String::UniquePtr);

private:
    std::unique_ptr<SpeechRecognition> m_pRecognizor;
    std::unique_ptr<TextToSpeech> m_pSpeech;
    std::shared_ptr<message::MessageTranslate<>> m_pTranslate;
};

#endif // APPLICATION_H
