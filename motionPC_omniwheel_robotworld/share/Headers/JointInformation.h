// This JointInformation.h is for DRC-HUBO+

#ifndef JOINT_INFORMATION_H
#define JOINT_INFORMATION_H

#include <QVector>
#include <QString>



enum JointSequentialNumber
{
    RHY = 0, RHR, RHP, RKN, BWH, RAR,
    LHY, LHR, LHP, LKN, LAP, LAR,
    RSP, RSR, RSY, REB, RWY, RWP,
    LSP, LSR, LSY, LEB, LWY, LWP,
    WST,
    RWY2, RHAND, LWY2, LHAND,
    RWH, LWH,
    NKP1, NKY, NKP2, NKR,
    RF1, RF2, RF3, RF4,
    LF1, LF2, LF3, LF4, NO_OF_JOINTS
};


const QString JointNameList[NO_OF_JOINTS] = {
    "RHY", "RHR", "RHP", "RKN", "BWH", "RAR",
    "LHY", "LHR", "LHP", "LKN", "LAP", "LAR",
    "RSP", "RSR", "RSY", "REB", "RWY", "RWP",
    "LSP", "LSR", "LSY", "LEB", "LWY", "LWP",
    "WST",
    "RWY2", "RHAND", "LWY2", "LHAND",
    "RWH", "LWH",
    "NKP1", "NKY", "NKP2", "NKR",
    "RF1", "RF2", "RF3", "RF4",
    "LF1", "LF2", "LF3", "LF4"
};

const struct {
    int id;
    int ch;
} MC_ID_CH_Pairs[NO_OF_JOINTS] = {
    {0,0}, {1,0}, {2,0}, {3,0}, {4,0}, {5,0},
    {6,0}, {7,0}, {8,0}, {9,0}, {10,0},{11,0},
    {13,0}, {14,0}, {15,0}, {15,1}, {16,0},{16,1},
    {17,0},{18,0},{19,0},{19,1},{20,0},{20,1},
    {12,0},
    {21,0},{21,1},{22,0},{22,1},
    {4,1}, {10,1},
    {23,0},{23,1},{24,0},{24,1},
    {25,0},{25,1},{26,0},{26,1},
    {27,0},{27,1},{28,0},{28,1}
};

inline int MC_GetID(int jnum){
    return MC_ID_CH_Pairs[jnum].id;
}
inline int MC_GetCH(int jnum){
    return MC_ID_CH_Pairs[jnum].ch;
}

#endif // JOINT_INFORMATION_H
