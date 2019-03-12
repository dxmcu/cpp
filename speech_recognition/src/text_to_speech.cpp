#include <cstring>
#include <functional>
#include "speech_recognition/text_to_speech.h"
#include "speech_recognition/qtts.h"
#include "speech_recognition/msp_cmn.h"
#include "speech_recognition/msp_errors.h"
#include "speech_recognition/application.h"

TextToSpeech::TextToSpeech(Application &rApplication)
    : m_rApplication(rApplication)
{

}

void TextToSpeech::Start()
{
    std::thread *pThread = new std::thread(std::mem_fn(&TextToSpeech::OnThreadToSpeech), this);
    m_pThreadToSpeech.reset(pThread);
}

void TextToSpeech::Stop()
{
    m_bFlagSpeech = false;
    m_pThreadToSpeech->join();
}

void TextToSpeech::PostTextMsg(const std::shared_ptr<SpeechMsg> msg)
{
    {
        std::lock_guard<std::mutex> lock(m_mutexMsg);
        m_listMsgText.push_back(msg);
    }

    m_cvMsg.notify_one();
}

std::shared_ptr<SpeechMsg> TextToSpeech::GetTextMsg()
{
    std::unique_lock<std::mutex> lock(m_mutexMsg);
    m_cvMsg.wait(lock, [this](){
        return m_listMsgText.size() > 0;
    });

    std::shared_ptr<SpeechMsg> msg;
    if (m_listMsgText.size() > 0)
    {
        msg = m_listMsgText.front();
        m_listMsgText.pop_front();
    }
    return msg;
}

void TextToSpeech::OnThreadToSpeech()
{
    m_bFlagSpeech = true;
    while (m_bFlagSpeech) {
        std::shared_ptr<SpeechMsg> msg = this->GetTextMsg();
        std::string speech;
        this->ToSpeech(msg->m_strText, speech);
        msg->m_strText = speech;
        m_rApplication.PublishMsg(Application::Publish_Speech, msg);
    }
}

int TextToSpeech::ToSpeech(const std::string &text, std::string &speech)
{
    int          ret          = -1;
    const char*  sessionID    = nullptr;
    unsigned int audio_len    = 0;
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;
    const char *params        = "engine_type = local,voice_name=xiaoyan, text_encoding = UTF8, "
            "tts_res_path = fo|res/tts/xiaoyan.jet;fo|res/tts/common.jet, sample_rate = 16000, "
            "speed = 50, volume = 50, pitch = 50, rdn = 2";

    if (text.empty())
    {
        printf("params is error!\n");
        return ret;
    }
    /* 开始合成 */
    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionBegin failed, error code: %d.\n", ret);
        return ret;
    }
    ret = QTTSTextPut(sessionID, text.c_str(), static_cast<unsigned int>(text.length()), nullptr);
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSTextPut failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "TextPutError");
        return ret;
    }
    printf("正在合成 ...\n");
    while (1)
    {
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
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSAudioGet failed, error code: %d.\n",ret);
        QTTSSessionEnd(sessionID, "AudioGetError");
        return ret;
    }
    /* 合成完毕 */
    ret = QTTSSessionEnd(sessionID, "Normal");
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionEnd failed, error code: %d.\n",ret);
    }

    return ret;
}

