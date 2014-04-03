//#include "wcomm.h"

void runclient(char *ip, char *fpath);
void runserver();


WComm w;

void runserver()
{
	// Start Server Daemon
	w.startServer(27017);
	printf("Server Started........\n");

    while (TRUE) {
        
		// Wait until a client connects
		w.waitForClient();
		printf("Client Connected......\n");

		// Work with client
		while(TRUE)
		{
			char rec[50] = "";
			//w.recvData(rec,32);w.sendData("OK");

			if(strcmp(rec,"FileSend")==0)
			{
				char fname[32] ="";
				w.fileReceive(fname);
				printf("File Received.........\n");
			}

			
			if(strcmp(rec,"EndConnection")==0)break;
			printf("Connection Ended......\n");

		}
		
		// Disconnect client
		w.closeConnection();
	}

}

void runclient(char *ip)
{
	char rec[32] = "";

	// Connect To Server
	w.connectServer(ip,27017);
	printf("Connected to server...\n");
	//w.sendData("FileSend");//	w.recvData(rec,32);

	/*
	// Send Close Connection Signal
	w.sendData("EndConnection");w.recvData(rec,32);
	printf("Connection ended......\n");
	*/
}

void senddata()
{
	while(1)
	{
		WaitForSingleObject(NewMap, INFINITE);
		EnterCriticalSection(&GrabSemaMap);
		memcpy(temp2BitMap,SemaMap,sizeof(temp2BitMap));
		memset(SemaMap,0,sizeof(SemaMap));
		LeaveCriticalSection(&GrabSemaMap);
		w.sendData(temp2BitMap,283);
	}
}