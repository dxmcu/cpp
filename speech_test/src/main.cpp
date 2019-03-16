#include <unistd.h>
#include <QApplication>
#include "qspeechwidget.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    std::string nodeName, publishTopic, subscribeTopic;
    int ch;
    while((ch = getopt(argc,argv,"p:s:"))!= -1)
    {
        switch (ch) {
        case 'p':
            publishTopic = optarg;
            break;
        case 's':
            subscribeTopic = optarg;
            break;
        default:
            break;
        }
    }

    QSpeechWidget w(nullptr, QString::fromStdString(publishTopic), QString::fromStdString(subscribeTopic));
    w.show();
    return app.exec();
}
