#include <QApplication>
#include <QLabel>
#include <QString>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QString message = QString::fromStdString("Hello, Qt!");
    QLabel * label = new QLabel(message);
    label->setText(message);
    label->show();
    
    app.exec();
    return 0;
}
