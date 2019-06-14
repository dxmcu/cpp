#ifndef TEXT_TO_SPEECH_H
#define TEXT_TO_SPEECH_H

#include <memory>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <list>
#include <atomic>

class Application;

struct SpeechMsg
{
    std::string m_strSender;
    std::string m_strText;
};

class TextToSpeech
{
public:
    TextToSpeech(Application &);

    void Start();
    void Stop();
    void PostTextMsg(const std::shared_ptr<SpeechMsg>);

protected:
    void OnThreadToSpeech();
    int ToSpeech(const std::string &, std::string &);

    std::shared_ptr<SpeechMsg> GetTextMsg();

private:
    std::unique_ptr<std::thread> m_pThreadToSpeech;
    std::atomic_bool m_bFlagSpeech;
    std::mutex m_mutexMsg;
    std::condition_variable m_cvMsg;
    std::list<std::shared_ptr<SpeechMsg>> m_listMsgText;
    Application &m_rApplication;
};

#endif // TEXT_TO_SPEECH_H
