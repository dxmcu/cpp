#ifndef SPEECH_RECOGNITION_H
#define SPEECH_RECOGNITION_H

#include <memory>
#include <thread>
#include <atomic>
#include "speech_recognition/speech_recognizer.h"

class Application;

class SpeechRecognition
{
public:
    SpeechRecognition(Application &);
    ~SpeechRecognition();

    void Start();
    void Restart();
    void Stop();

protected:
    void OnThreadRecognized();
    void DemoMic(const char*);

private:
    std::unique_ptr<std::thread> m_pThreadRecognizer;
    std::unique_ptr<std::thread> m_pThreadRestart;
    struct speech_rec iat;
    std::atomic_int m_nFlagRecognized; //0: start, 1: stop, 2: stop with restart

public:
    Application &m_rApplication;
};

#endif // SPEECH_RECOGNITION_H
