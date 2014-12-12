#include "afsocket.h"
#include "globals.h"

DWORD WINAPI MotionLoop(LPVOID param)
{
    socketstuff();
    return 0;
}

int socketstuff()
{

    char sendbuf[128] = {0};
    // Begin code to initialize Ethernet communication
    #if ETHERNET
        WSADATA wsaData;
        SOCKET ConnectSocket = INVALID_SOCKET;
        struct addrinfo *result = NULL, *ptr = NULL, hints;
        char *ipaddr = "192.168.1.52";
        char recvbuf[DEFAULT_BUFLEN];
        int iResult;
        int recvbuflen = DEFAULT_BUFLEN;

        // Initialize Winsock
        iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
        if (iResult != 0)
        {
            printf("WSAStartup failed: %d\n", iResult);
            return 1;
        }

        ZeroMemory(&hints, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;

        // Resolve the server address and port
        iResult = getaddrinfo(ipaddr, DEFAULT_PORT, &hints, &result);
        if ( iResult != 0 )
        {
            printf("getaddrinfo failed: %d\n", iResult);
            WSACleanup();
            return 1;
        }

        // Attempt to connect to an address until one succeeds
        for(ptr=result; ptr != NULL ;ptr=ptr->ai_next)
        {

            // Create a SOCKET for connecting to server
            ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
            if (ConnectSocket == INVALID_SOCKET)
            {
                printf("Error at socket(): %ld\n", (long)WSAGetLastError());
                freeaddrinfo(result);
                WSACleanup();
                return 1;
            }

            // Connect to server.
            iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
            if (iResult == SOCKET_ERROR)
            {
                closesocket(ConnectSocket);
                ConnectSocket = INVALID_SOCKET;
                continue;
            }
            break;
        }

        freeaddrinfo(result);

        if (ConnectSocket == INVALID_SOCKET)
        {
            printf("Unable to connect to server!\n");
            WSACleanup();
            return 1;
        }
    #endif
    // End code to initialize Ethernet communication
    while(!escape)      //while(!GetAsyncKeyState(VK_ESCAPE))
    {
        memset(sendbuf, 0, sizeof(sendbuf));
        //WaitForMultipleObjects() should tell us whether it is a sTermString or otherwise
        WaitForSingleObject(hNewMotion, INFINITE);
        //Now wait for the Semaphore (if sTermString)
        #if RECVDEBUG
            printf("Waiting for mTermString...\n");
        #endif
        WaitForSingleObject(mTermString, INFINITE);
        if (strlen(TermString))
        {
            #if MOTIONDEBUG
                printf("strlen(TermString): %d\n", strlen(TermString));
                printf("Terminally updating Galil...\n");
            #endif
            #if RECVDEBUG
                printf("Got mTermString...\n");
            #endif
            memcpy(sendbuf, TermString, MIN(sizeof(sendbuf), sizeof(TermString)));
            memset(TermString, '\0', sizeof(TermString));
            ReleaseMutex(mTermString);
            #if RECVDEBUG
                printf("Released mTermString...\n");
            #endif
        }
        else
        {
            #if MOTIONDEBUG
                printf("Autonomously updating Galil...\n");
            #endif
            ReleaseMutex(mTermString);
            long int mspeed;
            long int mdir;
            WaitForSingleObject(mVector, INFINITE);
            //formats the output, based on long int speed and long int dir given
            mspeed = speed;
            mdir = dir;
            //sprintf(sendbuf, "SPEED=%ld;DIR=%ld;BEGIN=1;", speed, (long)(-1.0*dir));
            ReleaseMutex(mVector);
            mspeed=50;
            printf("mspeed: %d\n", mspeed);
            printf("mdir: %d\n", mdir);

            sprintf(sendbuf, "JG,,,,%ld,%ld;", (long int)((float)(mspeed-3.0)*SPDSF+TURNF*sin((float)mdir*TORAD)), (long int)((float)(mspeed-3.0)*SPDSF-TURNF*sin((float)mdir*TORAD)));
            if (mspeed == 0 && mdir == 0)
            {
                SetEvent(hStopped);
                printf("Stopped ?\n");
            }
        }
        #if ETHERNETDEBUG
            printf("%s\n", sendbuf);
        #endif

        // Begin code to send/receive data via Ethernet
        #if ETHERNET
            // Send an initial buffer
            iResult = send( ConnectSocket, sendbuf, (int)strlen(sendbuf), 0 );
            if (iResult == SOCKET_ERROR)
            {
                printf("send failed: %d\n", WSAGetLastError());
                closesocket(ConnectSocket);
                WSACleanup();
                return 1;
            }

            //it should post the recvbuf: true or false
            bool postrecv = false;

            // Receive until the peer closes the connection
            do
            {
                memset(recvbuf, '\0', sizeof(recvbuf));
                iResult = recv(ConnectSocket, recvbuf, recvbuflen, 0);
                if ( iResult > 0 )
                {
                    //This code could cause it to slow down
                    #if RECVDEBUG
                        printf("Waiting for mRecvPtr...\n");
                    #endif
                    WaitForSingleObject(mRecvPtr, INFINITE);
                    #if RECVDEBUG
                        printf("Got mRecvPtr...\n");
                        printf("RecvPtr = %d\n", (int)RecvPtr);
                    #endif
                    if (RecvPtr)
                    {
                        postrecv = true;
                        #if RECVDEBUG
                            printf("recvbuf: %s\n", recvbuf);
                            printf("recvbuflen: %d\n", recvbuflen);         //I think recvbuflen == DEFAULT_BUFLEN
                        #endif
                        memcpy(RecvPtr, recvbuf, MIN(64, recvbuflen));      //64 was sizeof(RecvPtr), but should be ? sizeof(*RecvPtr)
                        RecvPtr = 0;
                        #if RECVDEBUG
                            printf("Updated mRecvPtr...\n");
                        #endif
                    }
                    ReleaseMutex(mRecvPtr);
                    #if RECVDEBUG
                        printf("Released mRecvPtr...\n");
                    #endif
                    if (postrecv)
                    {
                        //SetEvent(hNewRecv);
                    }
                    //End noting this code could cause it to slow down


                    //printf("Bytes received: %d\n", iResult);
                    //printf("Received: %s\n", recvbuf);
                    iResult = -1;
                }
                else if ( iResult == 0 )
                    printf("Connection closed\n");
                else
                    printf("recv failed: %d\n", WSAGetLastError());

            } while( iResult > 0 );

        #endif
        // End code to send/receive data via Ethernet
        mdelay(100);
        SetEvent(hMotionDone);
    }

    // Begin code to close Ethernet communication
    #if ETHERNET
        closesocket(ConnectSocket);
        WSACleanup();
    #endif
    // End code to close Ethernet communication

    return 0;
}


