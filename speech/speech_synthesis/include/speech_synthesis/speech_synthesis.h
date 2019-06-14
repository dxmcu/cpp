#ifndef SPEECH_SYNTHESIS_H
#define SPEECH_SYNTHESIS_H

#include <string>
#include <list>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

class SpeechSynthesis
{
public:
  SpeechSynthesis();

  bool login();
  void logout();
  void post_text(const std::string &);

protected:
  std::string get_text();
  void on_thread_to_speech();
  int text_to_speech(const std::string &, std::string &);

private:
  std::unique_ptr<std::thread> m_pThreadToSpeech;
  std::atomic_bool m_bFlagSpeech;
  std::mutex m_mutexMsg;
  std::condition_variable m_cvMsg;
  std::list<std::string> m_listText;
};

#endif // SPEECH_SYNTHESIS_H
