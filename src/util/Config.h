#pragma once
#include "CRGBA.h"
#include "mini/ini.h"

namespace util
{
	struct Config
	{
		bool ENABLE_BMX = false;
		bool ENABLE_MOVING = false;
		bool ENABLE_WATER_GPS = true;
		bool RESPECT_LANE_DIRECTION = true;
		bool ENABLE_DISTANCE_TEXT = true;
		bool DISTANCE_UNITS = false;
		bool ENABLE_ON_FOOT = true;
		bool ENABLE_OUTLINE = true;
		bool ENABLE_ETA = true;
		bool LOGFILE_ENABLED = false;
		bool ENABLE_CUSTOM_CLRS = false;

		// Animated line
		bool ENABLE_ANIM_LINE = true;
		float ANIM_SPEED = 3.0f;

		// Turn arrows
		bool ENABLE_TURN_ARROWS = true;
		float ARROW_SIZE = 3.5f;
		float TURN_ANGLE_THRESHOLD = 40.0f;

		// Next turn distance
		bool ENABLE_NEXT_TURN = true;

		// Off-route & Rerouting
		bool ENABLE_REROUTE = true;
		float OFF_ROUTE_DISTANCE = 45.0f;

		// Audio cues
		bool ENABLE_AUDIO = true;
		bool AUDIO_TURN_ALERT = true;
		bool AUDIO_ARRIVED = true;
		bool AUDIO_REROUTE = true;

		float GPS_LINE_WIDTH = 2.5f;
		float OUTLINE_WIDTH = 1.8f;
		float DISABLE_PROXIMITY = 40.0f;

		CRGBA CC_RED, CC_GREEN, CC_BLUE, CC_WHITE, CC_PURPLE, CC_YELLOW, CC_CYAN;
		CRGBA GPS_LINE_CLR = {180, 24, 24, 255};
		CRGBA OUTLINE_CLR = {0, 0, 0, 255};

		Config(const char *filename);
	};
} // namespace util