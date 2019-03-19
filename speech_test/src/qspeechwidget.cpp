#include <QBuffer>
#include <jsoncpp/json/json.h>
#include "qspeechwidget.h"
#include "ui_qspeechwidget.h"

QSpeechWidget::QSpeechWidget(QWidget *parent, const QString &publish, const QString &subscribe)
    : QWidget(parent)
    , ui(new Ui::QSpeechWidget)
{
    ui->setupUi(this);

    std::string str = publish.toStdString();
    std::string str2 = subscribe.toStdString();

    QAudioFormat nFormat;
    nFormat.setSampleRate(16000);
    nFormat.setSampleSize(50);
    nFormat.setChannelCount(2);
    nFormat.setCodec("audio/pcm");
    nFormat.setByteOrder(QAudioFormat::LittleEndian);
    nFormat.setSampleType(QAudioFormat::UnSignedInt);
    m_OutPut = new QAudioOutput(QAudioDeviceInfo::defaultOutputDevice(), nFormat);
    m_AudioIo = m_OutPut->start();

    std::string nodeName = "_node_speech_test_";
    m_pTranslate = std::make_shared<message::MessageTranslate<
            std_msgs::msg::String,
            std_msgs::msg::String>>(nodeName);
    m_pTranslate->CreatePublisher(str);

    auto fun = std::bind(&QSpeechWidget::OnSubsribe, this,
                         std::placeholders::_1);
    m_pTranslate->CreateSubscriber(str2, fun);
    m_pTranslate->Spin();
}

QSpeechWidget::~QSpeechWidget()
{
    m_pTranslate->Stop();
    delete ui;
}

void QSpeechWidget::OnSubsribe(std_msgs::msg::String::UniquePtr message)
{
    std::string text = message->data;

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(text, root)) return;

    Json::Value sender = root["sender"];
    Json::Value receiver = root["receiver"];
    Json::Value eventName = root["eventName"];
    Json::Value content = root["content"];
    std::string strSender = sender.asString();
    std::string strReceiver = receiver.asString();
    std::string strEventName = eventName.asString();

    if (strEventName == "TextToSpeech")
    {
        std::string strText = content.asString();
        if (strText.length() > 0)
        {
            QByteArray array = strText.data();
            QBuffer buffer(&array);
            buffer.open(QIODevice::ReadWrite);
            QAudioFormat nFormat;
            nFormat.setSampleRate(16000);
            nFormat.setSampleSize(50);
            nFormat.setChannelCount(2);
            nFormat.setCodec("audio/pcm");
            nFormat.setByteOrder(QAudioFormat::LittleEndian);
            nFormat.setSampleType(QAudioFormat::UnSignedInt);

            m_OutPut->start(&buffer);
        }
    }
    else if (strEventName == "SpeechRecognize")
    {
        std::string strText = content.asString();
        ui->textEdit->append(QString::fromStdString(strText));
    }
}

void QSpeechWidget::on_pushButton_clicked()
{
    Json::Value root;
    Json::Value content;
    root["sender"] = m_pTranslate->get_name();
    root["receiver"] = "";
    root["eventName"] = "TextToSpeech";
    QString strText = ui->textEdit_2->toPlainText();
    root["content"] = strText.toStdString();

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    std::string strMsg = Json::writeString(builder, root);

    auto message = std_msgs::msg::String();
    message.data = strMsg;
    m_pTranslate->PublishMsg(0, message);
}

