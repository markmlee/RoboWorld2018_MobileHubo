
#include "ExternHeader.h"
#include "RBProcessManager.h"
#include "RBDataBase.h"



RBProcessManager::RBProcessManager()
    : MAX_BUF(500), PID_LIST_BLOCK(32)
{
    // close previous processes if they are working
    CloseEveryALs();
    usleep(10*1000);

    int threadID = pthread_create(&ProcessHandler, NULL, &RBProcessThread, this);
    if(threadID < 0){
        FILE_LOG(logERROR) << "RBPM: Process thread Creation Error";
        isWorking = false;
    }
    isWorking = true;
}

void *RBProcessManager::RBProcessThread(void *_arg){
    RBProcessManager *rbPM = (RBProcessManager*)_arg;

    int whileCnt = 0;
    while(rbPM->isWorking){
        rbPM->CheckALWorking(++whileCnt%(_NO_OF_AL-1) + 1);
        usleep(200*1000);
    }
    return NULL;
}

int RBProcessManager::OpenAL(int no){
    UpdatePID(no);
    if(pidAL[no] != 0){
        if(CheckIsZombie(pidAL[no]) == false){
            FILE_LOG(logWARNING) << "AL[" << no << ": " << RBDataBase::_DB_AL[no].FileName.toStdString().data() << "] is already working..[PID: " << pidAL[no] << "]";
            return false;
        }
    }

    QString argAL, argGazebo, argROS;
    argAL.sprintf("-p %d", no);

    if(__IS_GAZEBO)
        argGazebo = "-gtrue";
    else
        argGazebo = "-gfalse";

    if(__IS_ROS)
        argROS = "-rtrue";
    else
        argROS = "-rfalse";

    sharedCMD->COMMAND[no].USER_COMMAND = NO_ACT;
    pid_t tempPID = fork();
    if(tempPID == 0){
        pidAL[no] = tempPID;
        FILE_LOG(logSUCCESS) << "AL[" << no << ": " << RBDataBase::_DB_AL[no].FileName.toStdString().data() << "] is working now..";
        QString tempName = MakeFullFileName(RBDataBase::_DB_AL[no].PathName, RBDataBase::_DB_AL[no].FileName);
        if(execl(tempName.toStdString().data(), tempName.toStdString().data(), argAL.toStdString().data(), argGazebo.toStdString().data(), argROS.toStdString().data(), NULL) == -1){
            FILE_LOG(logERROR) << "AL[" << no << ": " << RBDataBase::_DB_AL[no].FileName.toStdString().data() << "] creating error..";
            FILE_LOG(logERROR) << "Terminate this fork..";
            return -99;
        }
    }else if(tempPID == -1)
        FILE_LOG(logERROR) << "Fail to create process";

    return true;
}

void RBProcessManager::CloseAL(int no){
    UpdatePID(no);

    for(int i=0;i<_NO_OF_MC;i++){
        for(int j=0;j<_DEV_MC[i].MOTOR_CHANNEL;j++){
            if(sharedCMD->MotionOwner[i][j] == no){
                _DEV_MC[i].MoveJoints[j].RefAngleCurrent = sharedSEN->ENCODER[i][j].CurrentReference;
                sharedCMD->MotionOwner[i][j] = RBCORE_PODO_NO;
            }
        }
    }
    sharedCMD->COMMAND[no].USER_COMMAND = NO_ACT;

    if(pidAL[no] > 0){
        KillAL(no);
        FILE_LOG(logERROR) << "AL[" << no << ": " << RBDataBase::_DB_AL[no].FileName.toStdString().data() << "] is terminated..";
    }else{
        FILE_LOG(logERROR) << "AL[" << no << ": " << RBDataBase::_DB_AL[no].FileName.toStdString().data() << "] is not activated before..";
    }
}

void RBProcessManager::Finish(){
    isWorking = false;
}

int RBProcessManager::CheckALWorking(int no){
    UpdatePID(no);
    if(pidAL[no] <= 0){
        sharedSEN->PODO_AL_WORKING[no] = false;
        return false;
    }
    else{
        if(CheckIsZombie(pidAL[no])){
            pidAL[no] = 0;
            sharedSEN->PODO_AL_WORKING[no] = false;
            return false;
        }else{
            sharedSEN->PODO_AL_WORKING[no] = true;
            return true;
        }
    }
}

void RBProcessManager::CloseEveryALs(){
    for(int i=1; i<_NO_OF_AL; i++){
        UpdatePID(i);
        if(pidAL[i] > 0){
            FILE_LOG(logWARNING) << "Process(" << i << ") PID: " << pidAL[i] << "was worked..so killed";
            KillAL(i);
        }
    }
}

void RBProcessManager::UpdatePID(int no){
    int *list = PIDof(RBDataBase::_DB_AL[no].FileName.toStdString().data());
    int hasPID = false;
    for(int j=0; list[j]!=-1; j++){
        if(CheckIsZombie(list[j]) == false){
            pidAL[no] = list[j];
            hasPID = true;
            break;
        }
    }
    if(hasPID == false)
        pidAL[no] = 0;
    free(list);
}

int *RBProcessManager::PIDof(const char *pname){
    DIR *dirp;
    FILE *fp;
    struct dirent *entry;
    int *pidlist, pidlist_index = 0, pidlist_realloc_count = 1;
    char path[MAX_BUF], read_buf[MAX_BUF];

    dirp = opendir ("/proc/");
    if(dirp == NULL){
        perror ("Fail");
        return NULL;
    }

    pidlist = (int*)malloc(sizeof(int) * PID_LIST_BLOCK);

    while((entry = readdir(dirp)) != NULL){
        if(CheckIsNumber(entry->d_name)){
            strcpy(path, "/proc/");
            strcat(path, entry->d_name);
            strcat(path, "/comm");

            /* A file may not exist, it may have been removed.
            * dut to termination of the process. Actually we need to
            * make sure the error is actually file does not exist to
            * be accurate.
            */
            fp = fopen (path, "r");
            if(fp != NULL){
                fscanf(fp, "%s", read_buf);
                if(strcmp(read_buf, pname) == 0){
                    /* add to list and expand list if needed */
                    pidlist[pidlist_index++] = atoi (entry->d_name);
                    if(pidlist_index == PID_LIST_BLOCK * pidlist_realloc_count){
                        pidlist_realloc_count++;
                        pidlist = (int*)realloc(pidlist, sizeof (int) * PID_LIST_BLOCK * pidlist_realloc_count); //Error check todo
                    }
                }
                fclose(fp);
            }
        }
    }

    closedir(dirp);
    pidlist[pidlist_index] = -1; /* indicates end of list */
    return pidlist;
}

void RBProcessManager::KillAL(int i){
    kill(pidAL[i],SIGKILL);
    pidAL[i] = 0;
}

int RBProcessManager::CheckIsZombie(int pid){
    FILE *fp;
    char path[100], read_buf1[30], read_buf2[30], read_buf3[30], status;

    sprintf(path, "/proc/%d/status", pid);
    fp = fopen(path, "r");
    if(fp != NULL){
        fscanf(fp, "%s\t%s\n%s\t%c", read_buf1, read_buf2, read_buf3, &status);
        fclose(fp);
    }
    if(status == 'Z')
        return true;
    else
        return false;
}
int RBProcessManager::CheckIsNumber(char *str){
    for(int i=0; str[i] != '\0'; i++){
        if(!isdigit(str[i])){
            return 0;
        }
    }
    return 1;
}

QString RBProcessManager::MakeFullFileName(QString path, QString file){
    QFileInfo fileInfo;
    fileInfo.setFile(QDir(path), file);
    return fileInfo.absoluteFilePath();
}
