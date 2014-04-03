//=============================================================================
// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

//------------------------------------------------------------------------------
// File: PGRPlayCap.h
//
// Desc: DirectShow sample code - header file for FlyStreamCapEx previewer
//       application.  This code is based largely on the PlayCap Sample in the MS DDK.
//------------------------------------------------------------------------------


//
// Function prototypes
//
int PASCAL WinMain(HINSTANCE hInstance, HINSTANCE hInstP, LPSTR lpCmdLine, int nCmdShow);
LRESULT CALLBACK WndMainProc (HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

HRESULT GetInterfaces(void);
HRESULT CaptureVideo();
HRESULT FindCaptureDevice(IBaseFilter ** ppSrcFilter);
HRESULT SetupVideoWindow(void);
HRESULT ChangePreviewState(int nShow);
HRESULT HandleGraphEvent(void);
HRESULT SetFlyStreamCamera(IBaseFilter * pSrcFilter);

void Msg(TCHAR *szFormat, ...);
void CloseInterfaces(void);
void ResizeVideoWindow(void);

// Remote graph viewing functions
HRESULT AddGraphToRot(IUnknown *pUnkGraph, DWORD *pdwRegister);
void RemoveGraphFromRot(DWORD pdwRegister);

enum PLAYSTATE {Stopped, Paused, Running, Init};

//
// Macros
//
#define SAFE_RELEASE(x) { if (x) x->Release(); x = NULL; }

#define JIF(x) if (FAILED(hr=(x))) \
    {Msg(TEXT("FAILED(hr=0x%x) in ") TEXT(#x) TEXT("\n\0"), hr); return hr;}

//
// Constants
//
#define DEFAULT_VIDEO_WIDTH		640
#define DEFAULT_VIDEO_HEIGHT    480

#define APPLICATIONNAME TEXT("FlyStream Capture (PlayCap)\0")
#define CLASSNAME       TEXT("FlyStreamCapPreviewer\0")

// Application-defined message to notify app of filtergraph events
#define WM_GRAPHNOTIFY  WM_APP+1