#include <windows.h>
#include <iostream>
#include <string>

#include "pxcsensemanager.h"
#include "pxcmetadata.h"
#include "service/pxcsessionservice.h"

#include "pxchandmodule.h"
#include "pxchanddata.h"
#include "pxchandconfiguration.h"

#include "Definitions.h"

using namespace Intel::RealSense;
using namespace Intel::RealSense::Hand;

bool g_live			= true;	    // true - Working in live camera mode, false - sequence mode
bool g_gestures	= true;	    // Writing gesture data to console output
bool g_alerts		= false   ;	// Writing alerts data to console output
bool g_stop			= false ;	// user closes application

PXCSession				        *g_session;
PXCSenseManager			*g_senseManager;
PXCHandModule			    *g_handModule;
PXCHandData				    *g_handDataOutput;
PXCHandConfiguration	*g_handConfiguration;

void releaseAll()
{
    if (g_handConfiguration)
    {
        g_handConfiguration->Release();
        g_handConfiguration = NULL;
    }
    if (g_handDataOutput)
    {
        g_handDataOutput->Release();
        g_handDataOutput = NULL;
    }
    if (g_senseManager)
    {
        g_senseManager->Close();
        g_senseManager->Release();
        g_senseManager = NULL;
    }
    if (g_session)
    {
        g_session->Release();
        g_session = NULL;
    }
}

int main(int argc, const char* argv[])
{
	// Setup and check out hardware status
	g_session = PXCSession::CreateInstance();
	if (!g_session)
	{
		std::printf("Failed Creating PXCSession\n");
		return -1;
	}
	g_senseManager = g_session->CreateSenseManager();
	if (!g_senseManager)
	{
		releaseAll();
		std::printf("Failed Creating PXCSenseManager\n");
		return -1;
	}
	if (g_senseManager->EnableHand(0) != pxcStatus::PXC_STATUS_NO_ERROR)
	{
		releaseAll();
		std::printf("Failed Enabling Hand Module\n");
		return -1;
	}
	g_handModule = g_senseManager->QueryHand();
	if (!g_handModule)
	{
		releaseAll();
		std::printf("Failed Creating PXCHandModule\n");
		return -1;
	}
	g_handDataOutput = g_handModule->CreateOutput();
	if (!g_handDataOutput)
	{
		releaseAll();
		std::printf("Failed Creating PXCHandData\n");
		return -1;
	}
	g_handConfiguration = g_handModule->CreateActiveConfiguration();
	if (!g_handConfiguration)
	{
		releaseAll();
		std::printf("Failed Creating PXCHandConfiguration\n");
		return -1;
	}
	g_handConfiguration->SetTrackingMode(HandData::TRACKING_MODE_FULL_HAND);

    // output messages about gesture and alert
	if (g_gestures)
	{
		std::printf("-Gestures Are Enabled-\n");
		g_handConfiguration->EnableAllGestures();
	}
	if (g_alerts)
	{
		std::printf("-Alerts Are Enabled-\n");
		g_handConfiguration->EnableAllAlerts();
	}

	// Apply configuration setup
	g_handConfiguration->ApplyChanges();

	pxcI32 numOfHands = 0;

	// First Initializing the sense manager
	pxcStatus initStatus = g_senseManager->Init();
	if (initStatus == PXC_STATUS_NO_ERROR)
	{
		std::printf("\nPXCSenseManager Initializing OK\n========================\n");

		// Acquiring frames from input device
		while (g_senseManager->AcquireFrame(true) == PXC_STATUS_NO_ERROR && !g_stop)
		{
			// Get current hand outputs
			if (g_handDataOutput->Update() == PXC_STATUS_NO_ERROR)
			{
				// Display alerts
				if (g_alerts)
				{
					PXCHandData::AlertData alertData;
					for (int i = 0; i < g_handDataOutput->QueryFiredAlertsNumber(); ++i)
					{
						if (g_handDataOutput->QueryFiredAlertData(i, alertData) == PXC_STATUS_NO_ERROR)
							std::printf("%s was fired at frame %d \n", Definitions::AlertToString(alertData.label).c_str(), alertData.frameNumber);
					}
				}
				// Display gestures
				if (g_gestures)
                {

					PXCHandData::GestureData gestureData;
					int check = g_handDataOutput->QueryFiredGesturesNumber();
                    for (int i = 0; i < check; ++i)
                    {
                        if (g_handDataOutput->QueryFiredGestureData(i, gestureData) == PXC_STATUS_NO_ERROR)
                        {
                            PXCHandData::IHand* handData;
                            if (g_handDataOutput->QueryHandDataById(gestureData.handId, handData) == PXC_STATUS_NO_ERROR)
                            {
                                // convert wchar_t* to char*
                                char tempStateName[20];
                                WideCharToMultiByte(CP_ACP, 0, (wchar_t*)(Definitions::GestureStateToString(gestureData.state)), -1, tempStateName, sizeof(tempStateName), NULL, NULL);
                                char tempDataName[10];
                                WideCharToMultiByte(CP_ACP, 0, gestureData.name, -1, tempDataName, sizeof(tempDataName), NULL, NULL);

                                // judge left or right hand to decide downpage or uppage
                                if (handData->QueryBodySide() == PXCHandData::BodySideType::BODY_SIDE_LEFT)
                                {
                                    std::wprintf(L"[Left hand] %s, Gesture: %s was fired at frame %d \n", Definitions::GestureStateToString(gestureData.state), gestureData.name, gestureData.frameNumber);
                                    if (strcmp(tempStateName, "GESTURE_STATE_END") == 0 && strcmp(tempDataName, "click") == 0)
                                    {
                                        keybd_event(VK_LEFT, 0, 0, 0);
                                        Sleep(10);
                                        keybd_event(VK_LEFT, 0, KEYEVENTF_KEYUP, 0);
                                    }
                                }
                                if (handData->QueryBodySide() == PXCHandData::BodySideType::BODY_SIDE_RIGHT)
                                {
                                    std::wprintf(L"[Right hand] %s, Gesture: %s was fired at frame %d \n", Definitions::GestureStateToString(gestureData.state), gestureData.name, gestureData.frameNumber);
                                    if (strcmp(tempStateName, "GESTURE_STATE_END") == 0 && strcmp(tempDataName, "click") == 0)
                                    {
                                        keybd_event(VK_RIGHT, 0, 0, 0);
                                        Sleep(10);
                                        keybd_event(VK_RIGHT, 0, KEYEVENTF_KEYUP, 0);
                                    }
                                }
                            }
                        }
					}
				}
			}

			// Display number of hands
			if (numOfHands != g_handDataOutput->QueryNumberOfHands())
			{
				numOfHands = g_handDataOutput->QueryNumberOfHands();
				std::printf("Number of hands: %d\n", numOfHands);
			}

			g_senseManager->ReleaseFrame();
		} // end while acquire frame
	} // end if Init

	else
	{
		releaseAll();
		std::printf("Failed Initializing PXCSenseManager\n");
		return -1;
	}
	releaseAll();
    return 0;
}


