#ifndef RBPROCESSMANAGER_H
#define RBPROCESSMANAGER_H

#include <sys/types.h>
#include <ctype.h>
#include <stdlib.h>
#include <dirent.h>
#include <libgen.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>

#include <QFileInfo>
#include <QDir>

#include "RBDataType.h"
#include "RBLog.h"



class RBProcessManager
{
public:
    RBProcessManager();

    int     CheckALWorking(int no);
    void    CloseEveryALs();
    int     OpenAL(int no);
    void    CloseAL(int no);
    void    UpdatePID(int no);
    void    Finish();

private:
    int     *PIDof(const char *pname);
    int     CheckIsZombie(int pid);
    int     CheckIsNumber(char *str);
    void    KillAL(int i);
    QString MakeFullFileName(QString path, QString file);


    pid_t   pidAL[MAX_AL];
    ulong   ProcessHandler;
    static void *RBProcessThread(void *_arg);

    int         isWorking;
    const int   MAX_BUF;
    const int   PID_LIST_BLOCK;
};

#endif // RBPROCESSMANAGER_H
