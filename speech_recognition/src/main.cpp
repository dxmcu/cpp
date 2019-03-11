#include <iostream>
#include "speech_recognition/msp_cmn.h"
#include "speech_recognition/msp_errors.h"
#include "speech_recognition/speech_recognition.h"
#include "speech_recognition/text_to_speech.h"

int main(int argc, char ** argv)
{
    int         ret          = MSP_SUCCESS;
    const char* login_params = "appid = 5c7e2821, work_dir = ."; // 登录参数，appid与msc库绑定,请勿随意改动

    ret = MSPLogin(nullptr, nullptr, login_params); //第一个参数是用户名，第二个参数是密码，均传NULL即可，第三个参数是登录参数
    if (MSP_SUCCESS != ret)
    {
        std::cout << "msp login failed." << std::endl;
        exit(-1);
    }

    SpeechRecognition recognition;
    recognition.Start();

    TextToSpeech speech;
    speech.Start();

    getchar();
    recognition.Stop();
    speech.Stop();
    MSPLogout(); //退出登录

    return 0;
}
