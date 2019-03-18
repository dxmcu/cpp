#ifndef QSPEECHWIDGET_H
#define QSPEECHWIDGET_H

#include <QWidget>
#include <QAudioOutput>
#include "common/messagetranslate.hpp"

namespace Ui {
class QSpeechWidget;
}

class QSpeechWidget : public QWidget
{
    Q_OBJECT

public:
    explicit QSpeechWidget(QWidget *parent, const QString &, const QString &);
    ~QSpeechWidget();

protected:
    void OnSubsribe(std_msgs::msg::String::UniquePtr);
    void OnSubsribeTts(std_msgs::msg::String::UniquePtr);

private slots:
    void on_pushButton_clicked();

private:
    Ui::QSpeechWidget *ui;
    std::shared_ptr<message::MessageTranslate<>> m_pTranslate;

    QAudioOutput *m_OutPut;
    QIODevice *m_AudioIo;
};

#endif // QSPEECHWIDGET_H
