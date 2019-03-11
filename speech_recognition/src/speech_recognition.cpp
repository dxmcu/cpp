#include <cstring>
#include <thread>
#include <iostream>
#include <functional>
#include "speech_recognition/speech_recognition.h"
#include "speech_recognition/qisr.h"
#include "speech_recognition/msp_cmn.h"
#include "speech_recognition/msp_errors.h"

#define FRAME_LEN	640
#define	BUFFER_SIZE	4096

static void show_result(char *string, char is_over)
{
    printf("\rResult: [ %s ]", string);
    if(is_over)
        putchar('\n');
}

static SpeechRecognition *g_pRecognizer = nullptr;
static char *g_result = nullptr;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
    if (result) {
        size_t left = g_buffersize - 1 - strlen(g_result);
        size_t size = strlen(result);
        if (left < size) {
            g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
            if (g_result)
                g_buffersize += BUFFER_SIZE;
            else {
                printf("mem alloc failed\n");
                return;
            }
        }
        strncat(g_result, result, size);
        show_result(g_result, is_last);
    }
}
void on_speech_begin()
{
    if (g_result)
    {
        free(g_result);
    }
    g_result = (char*)malloc(BUFFER_SIZE);
    g_buffersize = BUFFER_SIZE;
    memset(g_result, 0, g_buffersize);

    printf("Start Listening...\n");
}
void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)
        printf("\nSpeaking done \n");
    else
        printf("\nRecognizer error %d\n", reason);

    if (nullptr != g_pRecognizer)
        g_pRecognizer->Restart();
}

SpeechRecognition::SpeechRecognition()
{
    g_pRecognizer = this;
}

SpeechRecognition::~SpeechRecognition()
{
}

void SpeechRecognition::Start()
{
    std::thread *pThread = new std::thread(std::mem_fn(&SpeechRecognition::OnThreadRecognized), this);
    m_pThreadRecognizer.reset(pThread);
}

void SpeechRecognition::Restart()
{
    if (m_nFlagRecognized != 1)
        m_nFlagRecognized = 2;
}

void SpeechRecognition::Stop()
{
    m_nFlagRecognized = 1;
    m_pThreadRecognizer->join();
}

void SpeechRecognition::OnThreadRecognized()
{
    do {
        m_nFlagRecognized = 0;
        const char* session_begin_params =
            "sub = iat, domain = iat, language = zh_cn, "
            "accent = mandarin, sample_rate = 16000, "
            "result_type = plain, result_encoding = utf8";

        DemoMic(session_begin_params);
    }while (m_nFlagRecognized == 2);
}

/* demo recognize the audio from microphone */
void SpeechRecognition::DemoMic(const char* session_begin_params)
{
    int errcode;

    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end
    };

    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode) {
        printf("speech recognizer init failed\n");
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode) {
        printf("start listen failed %d\n", errcode);
    }
    /* demo 15 seconds recording */
    while(m_nFlagRecognized == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (m_nFlagRecognized == 1)
    {
        errcode = sr_stop_listening(&iat);
        if (errcode) {
            printf("stop listen failed %d\n", errcode);
        }
    }
    sr_uninit(&iat);
}


