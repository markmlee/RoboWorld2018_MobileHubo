#include <QApplication>
#include <QCoreApplication>

#include <QString>
#include <QLockFile>
#include <QDir>
#include <QMessageBox>

#include "LauncherDialog.h"


int main(int argc, char *argv[])
{
    //QCoreApplication a(argc, argv);
    QApplication a(argc, argv);

    QString tempDir = QDir::tempPath();
    QLockFile lockFile(tempDir + "/JeongsooLim.lock");

    if(!lockFile.tryLock(100)){
        QMessageBox::warning(NULL, "Duplicate Running", "You already have this app running.\nOnly one instance is allowed.");
        return 0;
    }

    LauncherDialog launch;
    launch.show();

    return a.exec();
}
