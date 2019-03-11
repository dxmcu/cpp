#ifndef TEXT_TO_SPEECH_H
#define TEXT_TO_SPEECH_H

#include <memory>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <list>
#include <atomic>

class TextToSpeech
{
public:
    TextToSpeech();

    void Start();
    void Stop();

protected:
    void OnThreadToSpeech();
    int ToSpeech(const std::string &, std::string &);

    void PostTextMsg(const std::string &);
    std::string GetTextMsg();

private:
    std::unique_ptr<std::thread> m_pThreadToSpeech;
    std::atomic_bool m_bFlagSpeech;
    std::mutex m_mutexMsg;
    std::condition_variable m_cvMsg;
    std::list<std::string> m_listMsgText;
};

#endif // TEXT_TO_SPEECH_H
