#include <functional>
#include "speech_synthesis/speech_synthesis.h"
#include "speech_synthesis/xf/msp_cmn.h"
#include "speech_synthesis/xf/msp_errors.h"
#include "speech_synthesis/xf/qtts.h"

SpeechSynthesis::SpeechSynthesis()
{

}

bool SpeechSynthesis::login()
{
  int ret = MSP_SUCCESS;
  const char *login_params = "appid = 5c7e2821, work_dir = ."; // 登录参数，appid与msc库绑定,请勿随意改动
  ret = MSPLogin(nullptr, nullptr, login_params); //第一个参数是用户名，第二个参数是密码，均传NULL即可，第三个参数是登录参数

  if (MSP_SUCCESS == ret) {
    std::thread *pThread = new std::thread(std::mem_fn(&SpeechSynthesis::on_thread_to_speech), this);
    m_pThreadToSpeech.reset(pThread);
  }

  return (MSP_SUCCESS == ret);
}

void SpeechSynthesis::logout()
{
  m_bFlagSpeech = false;
  m_pThreadToSpeech->join();

  MSPLogout(); //退出登录
}

void SpeechSynthesis::post_text(const std::string &text)
{
  {
    std::lock_guard<std::mutex> lock(m_mutexMsg);
    m_listText.push_back(text);
  }

  m_cvMsg.notify_one();
}

std::string SpeechSynthesis::get_text()
{
  std::unique_lock<std::mutex> lock(m_mutexMsg);
  m_cvMsg.wait(lock, [this](){
    return m_listText.size() > 0;
  });

  std::string text;
  if (m_listText.size() > 0)
  {
    text = std::move(m_listText.front());
    m_listText.pop_front();
  }
  return text;
}

void SpeechSynthesis::on_thread_to_speech()
{
  m_bFlagSpeech = true;
  while (m_bFlagSpeech) {
    std::string &&text = this->get_text();
    std::string speech;
    this->text_to_speech(text, speech);
  }
}

int SpeechSynthesis::text_to_speech(const std::string &text, std::string &speech)
{
  int          ret          = -1;
  const char*  sessionID    = nullptr;
  unsigned int audio_len    = 0;
  int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;
  const char   *params      = "engine_type = local,voice_name=xiaoyan, text_encoding = UTF8, "
                              "tts_res_path = fo|res/tts/xiaoyan.jet;fo|res/tts/common.jet, sample_rate = 16000, "
                              "speed = 50, volume = 50, pitch = 50, rdn = 2";

  if (text.empty()) {
    printf("params is error!\n");
    return ret;
  }
  /* 开始合成 */
  sessionID = QTTSSessionBegin(params, &ret);
  if (MSP_SUCCESS != ret) {
    printf("QTTSSessionBegin failed, error code: %d.\n", ret);
    return ret;
  }
  ret = QTTSTextPut(sessionID, text.c_str(), static_cast<unsigned int>(text.length()), nullptr);
  if (MSP_SUCCESS != ret) {
    printf("QTTSTextPut failed, error code: %d.\n",ret);
    QTTSSessionEnd(sessionID, "TextPutError");
    return ret;
  }
  printf("正在合成: %s ...\n", text.c_str());
  while (1) {
    /* 获取合成音频 */
    const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
    if (MSP_SUCCESS != ret)
      break;
    if (nullptr != data)
    {
      const char *pData = static_cast<const char *>(data);
      speech.assign(pData, pData + audio_len);
    }
    if (MSP_TTS_FLAG_DATA_END == synth_status)
      break;
  }
  printf("\n");
  if (MSP_SUCCESS != ret) {
    printf("QTTSAudioGet failed, error code: %d.\n",ret);
    QTTSSessionEnd(sessionID, "AudioGetError");
    return ret;
  }
  /* 合成完毕 */
  ret = QTTSSessionEnd(sessionID, "Normal");
  if (MSP_SUCCESS != ret) {
    printf("QTTSSessionEnd failed, error code: %d.\n",ret);
  }

  return ret;
}


