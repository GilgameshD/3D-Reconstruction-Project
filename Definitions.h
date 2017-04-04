/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#pragma once
#include <iostream>
#include "pxcsensemanager.h"
#include "pxchanddata.h"
#include "pxchandconfiguration.h"

namespace Definitions
{
	static std::string appName = "";

	inline void WriteHelpMessage()
	{
		std::cout << "Usage in live streaming: " << appName.c_str() << " -live [optional parameters... (see below)] \n\n";
		std::cout << "Usage in sequence streaming: " << appName.c_str() << " -seq <seqPath> [optional parameters... (see below)] \n\n";

		std::cout << "Optional Parameters:\n\n";

		std::cout << "  -info			Prints out joint information\n\n";
		std::cout << "  -alerts		Prints out fired alert information \n\n";
		std::cout << "  -gestures		Prints out fired gesture information \n\n";
		std::cout << "  -fps			Prints out frames per second \n\n";
	}

	inline const std::string AlertToString(PXCHandData::AlertType label)
	{
		std::string alertLabel = "";
		switch (label)
		{
		case PXCHandData::AlertType::ALERT_HAND_DETECTED: {alertLabel = "ALERT_HAND_DETECTED"; break; }
		case PXCHandData::AlertType::ALERT_HAND_NOT_DETECTED: {alertLabel = "ALERT_HAND_NOT_DETECTED"; break; }
		case PXCHandData::AlertType::ALERT_HAND_TRACKED: {alertLabel = "ALERT_HAND_TRACKED"; break; }
		case PXCHandData::AlertType::ALERT_HAND_NOT_TRACKED: {alertLabel = "ALERT_HAND_NOT_TRACKED"; break; }
		case PXCHandData::AlertType::ALERT_HAND_CALIBRATED: {alertLabel = "ALERT_HAND_CALIBRATED"; break; }
		case PXCHandData::AlertType::ALERT_HAND_NOT_CALIBRATED: {alertLabel = "ALERT_HAND_NOT_CALIBRATED"; break; }
		case PXCHandData::AlertType::ALERT_HAND_OUT_OF_BORDERS: {alertLabel = "ALERT_HAND_OUT_OF_BORDERS"; break; }
		case PXCHandData::AlertType::ALERT_HAND_INSIDE_BORDERS: {alertLabel = "ALERT_HAND_INSIDE_BORDERS"; break; }
		case PXCHandData::AlertType::ALERT_HAND_OUT_OF_LEFT_BORDER: {alertLabel = "ALERT_HAND_OUT_OF_LEFT_BORDER"; break; }
		case PXCHandData::AlertType::ALERT_HAND_OUT_OF_RIGHT_BORDER: {alertLabel = "ALERT_HAND_OUT_OF_RIGHT_BORDER"; break; }
		case PXCHandData::AlertType::ALERT_HAND_OUT_OF_TOP_BORDER: {alertLabel = "ALERT_HAND_OUT_OF_TOP_BORDER"; break; }
		case PXCHandData::AlertType::ALERT_HAND_OUT_OF_BOTTOM_BORDER: {alertLabel = "ALERT_HAND_OUT_OF_BOTTOM_BORDER"; break; }
		case PXCHandData::AlertType::ALERT_HAND_TOO_FAR: {alertLabel = "ALERT_HAND_TOO_FAR"; break; }
		case PXCHandData::AlertType::ALERT_HAND_TOO_CLOSE: {alertLabel = "ALERT_HAND_TOO_CLOSE"; break; }
		case PXCHandData::AlertType::ALERT_HAND_LOW_CONFIDENCE: {alertLabel = "ALERT_HAND_LOW_CONFIDENCE"; break; }
		}
		return alertLabel;
	}

	inline const std::string JointToString(PXCHandData::JointType label)
	{
		std::string jointLabel = "";
		switch (label)
		{
		    case PXCHandData::JointType::JOINT_WRIST: {jointLabel = "JOINT_WRIST"; break; }
		    case PXCHandData::JointType::JOINT_CENTER: {jointLabel = "JOINT_CENTER"; break; }
		    case PXCHandData::JointType::JOINT_THUMB_BASE: {jointLabel = "JOINT_THUMB_BASE"; break; }
		    case PXCHandData::JointType::JOINT_THUMB_JT1: {jointLabel = "JOINT_THUMB_JT1"; break; }
		    case PXCHandData::JointType::JOINT_THUMB_JT2: {jointLabel = "JOINT_THUMB_JT2"; break; }
		    case PXCHandData::JointType::JOINT_THUMB_TIP: {jointLabel = "JOINT_THUMB_TIP"; break; }
		    case PXCHandData::JointType::JOINT_INDEX_BASE: {jointLabel = "JOINT_INDEX_BASE"; break; }
		    case PXCHandData::JointType::JOINT_INDEX_JT1: {jointLabel = "JOINT_INDEX_JT1"; break; }
		    case PXCHandData::JointType::JOINT_INDEX_JT2: {jointLabel = "JOINT_INDEX_JT2"; break; }
		    case PXCHandData::JointType::JOINT_INDEX_TIP: {jointLabel = "JOINT_INDEX_TIP"; break; }
		    case PXCHandData::JointType::JOINT_MIDDLE_BASE: {jointLabel = "JOINT_MIDDLE_BASE"; break; }
		    case PXCHandData::JointType::JOINT_MIDDLE_JT1: {jointLabel = "JOINT_MIDDLE_JT1"; break; }
		    case PXCHandData::JointType::JOINT_MIDDLE_JT2: {jointLabel = "JOINT_MIDDLE_JT2"; break; }
		    case PXCHandData::JointType::JOINT_MIDDLE_TIP: {jointLabel = "JOINT_MIDDLE_TIP"; break; }
		    case PXCHandData::JointType::JOINT_RING_BASE: {jointLabel = "JOINT_RING_BASE"; break; }
		    case PXCHandData::JointType::JOINT_RING_JT1: {jointLabel = "JOINT_RING_JT1"; break; }
		    case PXCHandData::JointType::JOINT_RING_JT2: {jointLabel = "JOINT_RING_JT2"; break; }
		    case PXCHandData::JointType::JOINT_RING_TIP: {jointLabel = "JOINT_RING_TIP"; break; }
		    case PXCHandData::JointType::JOINT_PINKY_BASE: {jointLabel = "JOINT_PINKY_BASE"; break; }
		    case PXCHandData::JointType::JOINT_PINKY_JT1: {jointLabel = "JOINT_PINKY_JT1"; break; }
		    case PXCHandData::JointType::JOINT_PINKY_JT2: {jointLabel = "JOINT_PINKY_JT2"; break; }
		    case PXCHandData::JointType::JOINT_PINKY_TIP: {jointLabel = "JOINT_PINKY_TIP"; break; }
		}
		return jointLabel;
	}

	inline const pxcCHAR* GestureStateToString(PXCHandData::GestureStateType label)
	{
		pxcCHAR* gestureState = L"";
		switch (label)
		{
		case PXCHandData::GESTURE_STATE_START: {gestureState = L"GESTURE_STATE_START"; break; }
		case PXCHandData::GESTURE_STATE_IN_PROGRESS: {gestureState = L"GESTURE_STATE_IN_PROGRESS"; break; }
		case PXCHandData::GESTURE_STATE_END: {gestureState = L"GESTURE_STATE_END"; break; }
		}
		return gestureState;
	}

}