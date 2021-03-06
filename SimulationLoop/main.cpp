#include <stdio.h>
#include <Windows.h>
#include "Game.h"
#include "Camera.h"
#include "Math\Vector3D.h"
#include "Constants.h"

//const char TITLE[] = "Window Creation";
auto TITLE = L"Window Creation";

Game *game;

//********************OpenGL Initialization********************************
#include <gl\gl.h>                                // Header File For The OpenGL32 Library
#include <gl\glu.h>                               // Header File For The GLu32 Library
HDC			hDC=NULL;		// Private GDI Device Context
//*****************************************************************************

//************************ MESSAGE HANDLER **************************

LRESULT CALLBACK WindowProc(HWND hwnd, 
	UINT msg, 
	WPARAM wparam, 
	LPARAM lparam)
{
	// this is the main message handler of the system
	PAINTSTRUCT		ps;	// used in WM_PAINT
	HDC				hdc;	// handle to a device context

	// what is the message 
	switch(msg)
	{	
	case WM_CREATE: 
		{
			// do initialization stuff here
			return(0);
		} 

	case WM_PAINT: 
		{
			// validate the window
			hdc = BeginPaint(hwnd,&ps);	 
			EndPaint(hwnd,&ps);
			return(0);
		} 

	case WM_KEYDOWN:
		{
			// Handle any non-accelerated key commands
			switch (wparam)
			{
			case VK_ESCAPE:
			case VK_F12:
				PostMessage(hwnd, WM_CLOSE, 0, 0);
				return (0);
			case VK_W:
				game->camera->ProcessKeyboard(UP);
				break;
			case VK_S:
				game->camera->ProcessKeyboard(DOWN);
				break;
			case VK_D:
				game->camera->ProcessKeyboard(RIGHT);
				break;
			case VK_A:
				game->camera->ProcessKeyboard(LEFT);
				break;
			case VK_UP:
				game->camera->ProcessKeyboard(FORWARD);
				break;
			case VK_DOWN:
				game->camera->ProcessKeyboard(BACKWARD);
				break;
			case VK_1:
				// add ball
				game->SpawnBall();
				break;
			case VK_2:
				// add cube
				game->SpawnOBB();
				break;
			case VK_R:
				// reset
				game->Reset();
				break;
			case VK_N:
				// toggle net on/off
				break;
			case VK_P:
				// pause
				game->PauseResume();
				break;
			case VK_U:
				// increase time scale
				game->IncreaseTimeScale();
				break;
			case VK_J:
				// decrease time scale
				game->DecreaseTimeScale();
				break;
			case VK_I:
				// increase friction 
				game->IncreaseFriction();
				break;
			case VK_K:
				// decrease friction (min 0)
				game->DecreaseFriction();
				break;
			case VK_T:
				// increase ball size (max 0.9)
				game->IncreaseBallSize();
				break;
			case VK_B:
				// decrease ball size
				game->DecreaseBallSize();
				break;
			case VK_O:
				// increase elaticity (0.1 to 1.0)
				game->IncreaseRestitution();
				break;
			case VK_L:
				// decrease elaticity (0.1 to 1.0)
				game->DecreaseRestitution();
				break;
			case VK_F1:				
				game->ToggleDebugMode();
				break;
			case VK_F2:
				game->ToggleDebugBoard();
				break;
			}
			break;
		}

	case WM_DESTROY: 
		{
			// kill the application			
			PostQuitMessage(0);

			return(0);
		}

	default:
		break;

	} // end switch

	// process any messages that we didn't take care of 
	return (DefWindowProc(hwnd, msg, wparam, lparam));

} // end WinProc



//**************************Setup OpenGL***********************
void InitializeOpenGL(HWND hwnd, int width, int height)
{ 
	GLuint		PixelFormat;			// Holds The Results After Searching For A Match
	HGLRC		hRC=NULL;		// Permanent Rendering Context

	static	PIXELFORMATDESCRIPTOR pfd=				// pfd Tells Windows How We Want Things To Be
	{
		sizeof(PIXELFORMATDESCRIPTOR),				// Size Of This Pixel Format Descriptor
		1,											// Version Number
		PFD_DRAW_TO_WINDOW |						// Format Must Support Window
		PFD_SUPPORT_OPENGL |						// Format Must Support OpenGL
		PFD_DOUBLEBUFFER,							// Must Support Double Buffering
		PFD_TYPE_RGBA,								// Request An RGBA Format
		24,										    // Select Our Color Depth
		0, 0, 0, 0, 0, 0,							// Color Bits Ignored
		0,											// No Alpha Buffer
		0,											// Shift Bit Ignored
		0,											// No Accumulation Buffer
		0, 0, 0, 0,									// Accumulation Bits Ignored
		16,											// 16Bit Z-Buffer (Depth Buffer)  
		0,											// No Stencil Buffer
		0,											// No Auxiliary Buffer
		PFD_MAIN_PLANE,								// Main Drawing Layer
		0,											// Reserved
		0, 0, 0										// Layer Masks Ignored
	};

	if (!(hDC=GetDC(hwnd)))							// Did We Get A Device Context?
	{
		MessageBox(NULL,L"Can't Create A GL Device Context.",L"ERROR",MB_OK|MB_ICONEXCLAMATION);
	}

	if (!(PixelFormat=ChoosePixelFormat(hDC,&pfd)))	// Did Windows Find A Matching Pixel Format?
	{
		MessageBox(NULL,L"Can't Find A Suitable PixelFormat.",L"ERROR",MB_OK|MB_ICONEXCLAMATION);
	}

	if(!SetPixelFormat(hDC,PixelFormat,&pfd))		// Are We Able To Set The Pixel Format?
	{
		MessageBox(NULL,L"Can't Set The PixelFormat.",L"ERROR",MB_OK|MB_ICONEXCLAMATION);
	}

	if (!(hRC=wglCreateContext(hDC)))				// Are We Able To Get A Rendering Context?
	{
		MessageBox(NULL,L"Can't Create A GL Rendering Context.",L"ERROR",MB_OK|MB_ICONEXCLAMATION);
	}

	if(!wglMakeCurrent(hDC,hRC))					// Try To Activate The Rendering Context
	{
		MessageBox(NULL,L"Can't Activate The GL Rendering Context.",L"ERROR",MB_OK|MB_ICONEXCLAMATION);
	}

	ShowWindow(hwnd,SW_SHOW);						// Show The Window
	SetForegroundWindow(hwnd);						// Slightly Higher Priority
	SetFocus(hwnd);									// Sets Keyboard Focus To The Window

	if (height==0)										// Prevent A Divide By Zero By
	{
		height=1;										// Making Height Equal One
	}

	glViewport(0,0,width,height);						// Reset The Current Viewport

	glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
	glLoadIdentity();									// Reset The Projection Matrix

	// Calculate The Aspect Ratio Of The Window
	gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,0.1f,1000.0f);
	//glOrtho(-25,25,-25,25,-10, 10);

	glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
	glLoadIdentity();									// Reset The Modelview Matrix

	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearColor(0.5f, 0.5f, 1.f, 1.f);   				// Black Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
}



int main(HINSTANCE hInstance, HINSTANCE hPrevInstance,LPSTR lpCmdLine, int nCmdShow)
{
	return WinMain(hInstance, hPrevInstance, lpCmdLine, nCmdShow);
}


//************************ WIN MAIN***********************

int WINAPI WinMain( HINSTANCE hinstance,
	HINSTANCE hprevinstance,
	LPSTR lpcmdline,
	int ncmdshow)
{
	WNDCLASS	winclass;	// this will hold the class we create
	HWND		hwnd;		// generic window handle
	MSG			msg;		// generic message

	// first fill in the window class stucture
	winclass.style			= CS_HREDRAW | CS_VREDRAW;                  
	winclass.lpfnWndProc	= WindowProc;
	winclass.cbClsExtra		= 0;
	winclass.cbWndExtra		= 0;
	winclass.hInstance		= hinstance;
	winclass.hIcon			= LoadIcon(NULL, IDI_APPLICATION);
	winclass.hCursor		= LoadCursor(NULL, IDC_ARROW);
	winclass.hbrBackground  = (HBRUSH)GetStockObject(BLACK_BRUSH);
	winclass.lpszMenuName	= NULL;
	winclass.lpszClassName	= L"WindowCreation";

	// register the window class
	if (!RegisterClass(&winclass))
		return(0);


	// create the window
	if (!(hwnd = CreateWindow(L"WindowCreation", // class
		TITLE,	     // title
		WS_OVERLAPPEDWINDOW | WS_VISIBLE,
		0,
		0,
		//Set the size of the window to the size of the screen 
		WND_WIDTH,
		WND_HEIGHT,
		//GetSystemMetrics(SM_CXSCREEN),
		//GetSystemMetrics(SM_CYSCREEN),
		NULL,	   // handle to parent 
		NULL,	   // handle to menu
		hinstance,	// instance
		NULL)))	// creation parms
		return(0);

	InitializeOpenGL(hwnd, WND_WIDTH, WND_HEIGHT);

	game = new Game(hDC);
	if (game->Init())
	{
		// fixed timestep
		const float dt = 1.f / 60.f;

		// enter main event loop
		bool quit = false;
		while(!quit)
		{
			if (PeekMessage(&msg,NULL,0,0,PM_REMOVE))
			{ 
				// test if this is a quit
				if (msg.message == WM_QUIT) quit = true;

				// translate any accelerator keys
				TranslateMessage(&msg);
				// send the message to the window proc
				DispatchMessage(&msg);
			} // end if
			else {
				game->Update(dt);
			}

		} // end while
	}

	delete game;

	// return to Windows like this
	return(msg.wParam);

} // end WinMain


//************************ END OF WIN MAIN ***********************
