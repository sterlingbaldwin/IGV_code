#pragma once
#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

#include <windows.h>
#include <iostream>

#define MIN(X, Y)  ((X) < (Y) ? (X) : (Y))

using namespace std;

#define H_RTS 1
#define H_CTS 2

class CommDevice
{
public:
    CommDevice(char* commName, char HFlags = 0, unsigned long baud = 19200, unsigned int ReadSize = 128, char EventChar = '\n');
    ~CommDevice();
    void Read(char* output = NULL/*, unsigned int OSZ = 128*/);
    void Write(char* WriteBuffer);
    HANDLE hComm;
private:
    DCB m_dcb;
    class WriteVars
    {
    public:
        DWORD m_BytesDone;
        DWORD m_BytesToDo;
    } m_Write;
    class ReadVars : public WriteVars
    {
    public:
        char* m_Buffer;
        unsigned int m_Size;
    } m_Read;
};


#endif // SERIAL_H_INCLUDED
