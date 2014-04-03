#include "serial.h"

CommDevice::CommDevice(char* commName, char HFlags, unsigned long baud, unsigned int ReadSize, char EventChar)
{
    m_Read.m_Size = ReadSize;
    m_Read.m_Buffer = new char[ReadSize];
    memset(m_Read.m_Buffer, 0, m_Read.m_Size);
    // Open the Comm Port
    hComm = CreateFile(commName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    // Check for errors opening
    if (hComm == INVALID_HANDLE_VALUE)
    {
        DWORD errnum = GetLastError();
        cerr << "CreateFile Error: " << errnum << ": Unable to open " << commName << endl;
    }
    // Set up the Configuration
    m_dcb.DCBlength = sizeof(m_dcb);
    if (!GetCommState(hComm, &m_dcb))
	{
	    DWORD errnum = GetLastError();
		cerr << "GetCommState Error " << errnum << ": Unable to extract current state for " << commName << endl;
	}
    m_dcb.BaudRate = baud;
    m_dcb.ByteSize = 8;
    m_dcb.Parity = 0;
    m_dcb.StopBits = ONESTOPBIT;
    m_dcb.fNull = TRUE;
    m_dcb.EvtChar = EventChar;
    m_dcb.fOutxCtsFlow = (HFlags & H_CTS) ? TRUE : FALSE;
    m_dcb.fRtsControl = (HFlags & H_RTS) ? RTS_CONTROL_ENABLE : RTS_CONTROL_DISABLE;
    // Set the Configuration
    if (!SetCommState(hComm, &m_dcb))
    {
        DWORD errnum = GetLastError();
        cerr << "SetCommState Error " << errnum << ": Unable to configure " << commName << endl;// << "Error code: " << itoa(Error) << endl;
    }
    SetCommMask(hComm, EV_RXFLAG | EV_TXEMPTY);
}

CommDevice::~CommDevice()
{
    CloseHandle(hComm);
}

void CommDevice::Write(char* WriteBuffer)
{
    WriteFile(hComm, WriteBuffer, strlen(WriteBuffer), &(m_Write.m_BytesDone), NULL);
    cout << "O " << m_Write.m_BytesDone << ": " << WriteBuffer << endl;
}

void CommDevice::Read(char *output/*, unsigned int osz*/)
{
    memset(m_Read.m_Buffer, 0, m_Read.m_Size);
    //printf("starting serial read\n");
    ReadFile(hComm, m_Read.m_Buffer, m_Read.m_Size, &(m_Read.m_BytesDone), NULL);
    //printf("serial read done\n");
    if (m_Read.m_BytesDone)
    {
        if (output)
        {
            memcpy(output, m_Read.m_Buffer, MIN(m_Read.m_Size, m_Read.m_BytesDone));
            //cout << m_Read.m_BytesDone << ": " << m_Read.m_Buffer << endl;
        }
        else
        {
            cout << "I " << m_Read.m_BytesDone << ": " << m_Read.m_Buffer << endl;
        }
        m_Read.m_Buffer[m_Read.m_BytesDone-1] = 0;
        /*
        char* substr = strrchr(m_Read.m_Buffer, '!');
        substr = substr ? substr+1 : m_Read.m_Buffer;
        */
        memset(m_Read.m_Buffer, 0, m_Read.m_Size);
    }
}
