#include "Config.h"
#include <filesystem>
#include "plugin.h"

namespace util
{
	void ExtractColorFromString(std::string in, CRGBA &out)
	{
		if (in.empty())
			return;

		// Remove whitespace
		in.erase(std::ranges::remove_if(in, isspace).begin(), in.end());

		bool didR = false, didG = false, didB = false, didA = false;

		size_t pos = 0;
		for (unsigned char i = 0; i < 4; i++)
		{
			pos = in.find(',');

			if (!didR)
			{
				out.r = static_cast<unsigned char>(std::stoi(in.substr(0, pos)));
				didR = true;
			}
			else if (!didG)
			{
				out.g = static_cast<unsigned char>(std::stoi(in.substr(0, pos)));
				didG = true;
			}
			else if (!didB)
			{
				out.b = static_cast<unsigned char>(std::stoi(in.substr(0, pos)));
				didB = true;
			}
			else if (!didA)
			{
				out.a = (unsigned char)std::stoi(in.substr(0, pos).c_str());
				didA = true;
			}

			in.erase(0, pos + 1);
		}
	}

	Config::Config(const char *filename)
	{
		std::string resolvedPath = filename;
		if (!std::filesystem::exists(resolvedPath))
		{
			std::string pluginRel = PLUGIN_PATH(filename);
			if (std::filesystem::exists(pluginRel))
				resolvedPath = pluginRel;
		}

		mINI::INIFile file(resolvedPath);

		mINI::INIStructure ini;
		file.read(ini);

		/* Navigation */
		if (ini["Navigation"].has("respectTrafficLaneDirection"))
			RESPECT_LANE_DIRECTION = static_cast<bool>(std::atoi(ini["Navigation"]["respectTrafficLaneDirection"].c_str()));
		if (ini["Navigation"].has("lineWidth"))
			GPS_LINE_WIDTH = static_cast<float>(std::atof(ini["Navigation"]["lineWidth"].c_str()));
		if (ini["Navigation"].has("enableOutline"))
			ENABLE_OUTLINE = static_cast<bool>(std::atoi(ini["Navigation"]["enableOutline"].c_str()));
		if (ini["Navigation"].has("outlineWidth"))
			OUTLINE_WIDTH = static_cast<float>(std::atof(ini["Navigation"]["outlineWidth"].c_str()));
		if (ini["Navigation"].has("enableOnFoot"))
			ENABLE_ON_FOOT = static_cast<bool>(std::atoi(ini["Navigation"]["enableOnFoot"].c_str()));
		if (ini["Navigation"].has("enableOnBicycles"))
			ENABLE_BMX = static_cast<bool>(std::atoi(ini["Navigation"]["enableOnBicycles"].c_str()));
		if (ini["Navigation"].has("enableOnBoats"))
			ENABLE_WATER_GPS = static_cast<bool>(std::atoi(ini["Navigation"]["enableOnBoats"].c_str()));
		if (ini["Navigation"].has("trackMovingTargets"))
			ENABLE_MOVING = static_cast<bool>(std::atoi(ini["Navigation"]["trackMovingTargets"].c_str()));
		if (ini["Navigation"].has("removeRadius"))
			DISABLE_PROXIMITY = static_cast<float>(std::atof(ini["Navigation"]["removeRadius"].c_str()));

		/* Extras */
		if (ini["Extras"].has("displayDistance"))
			ENABLE_DISTANCE_TEXT = static_cast<bool>(std::atoi(ini["Extras"]["displayDistance"].c_str()));
		if (ini["Extras"].has("distanceUnits"))
			DISTANCE_UNITS = static_cast<bool>(std::atoi(ini["Extras"]["distanceUnits"].c_str()));
		if (ini["Extras"].has("displayETA"))
			ENABLE_ETA = static_cast<bool>(std::atoi(ini["Extras"]["displayETA"].c_str()));
		if (ini["Extras"].has("animatedLine"))
			ENABLE_ANIM_LINE = static_cast<bool>(std::atoi(ini["Extras"]["animatedLine"].c_str()));
		if (ini["Extras"].has("animationSpeed"))
			ANIM_SPEED = static_cast<float>(std::atof(ini["Extras"]["animationSpeed"].c_str()));
		if (ini["Extras"].has("turnArrows"))
			ENABLE_TURN_ARROWS = static_cast<bool>(std::atoi(ini["Extras"]["turnArrows"].c_str()));
		if (ini["Extras"].has("arrowSize"))
			ARROW_SIZE = static_cast<float>(std::atof(ini["Extras"]["arrowSize"].c_str()));
		if (ini["Extras"].has("turnAngleThreshold"))
			TURN_ANGLE_THRESHOLD = static_cast<float>(std::atof(ini["Extras"]["turnAngleThreshold"].c_str()));
		if (ini["Extras"].has("nextTurnDisplay"))
			ENABLE_NEXT_TURN = static_cast<bool>(std::atoi(ini["Extras"]["nextTurnDisplay"].c_str()));
		if (ini["Extras"].has("rerouteOnDeviate"))
			ENABLE_REROUTE = static_cast<bool>(std::atoi(ini["Extras"]["rerouteOnDeviate"].c_str()));
		if (ini["Extras"].has("offRouteDistance"))
			OFF_ROUTE_DISTANCE = static_cast<float>(std::atof(ini["Extras"]["offRouteDistance"].c_str()));
		if (ini["Extras"].has("audioCues"))
			ENABLE_AUDIO = static_cast<bool>(std::atoi(ini["Extras"]["audioCues"].c_str()));
		if (ini["Extras"].has("audioTurnAlert"))
			AUDIO_TURN_ALERT = static_cast<bool>(std::atoi(ini["Extras"]["audioTurnAlert"].c_str()));
		if (ini["Extras"].has("audioArrived"))
			AUDIO_ARRIVED = static_cast<bool>(std::atoi(ini["Extras"]["audioArrived"].c_str()));
		if (ini["Extras"].has("audioReroute"))
			AUDIO_REROUTE = static_cast<bool>(std::atoi(ini["Extras"]["audioReroute"].c_str()));

		/* Custom Colors */
		if (ini["Custom Colors"].has("enabled"))
			ENABLE_CUSTOM_CLRS = static_cast<bool>(std::atoi(ini["Custom Colors"]["enabled"].c_str()));
		if (ENABLE_CUSTOM_CLRS)
		{
			ExtractColorFromString(ini["Custom Colors"]["waypoint"], GPS_LINE_CLR);
			ExtractColorFromString(ini["Custom Colors"]["red"], CC_RED);
			ExtractColorFromString(ini["Custom Colors"]["green"], CC_GREEN);
			ExtractColorFromString(ini["Custom Colors"]["blue"], CC_BLUE);
			ExtractColorFromString(ini["Custom Colors"]["white"], CC_WHITE);
			ExtractColorFromString(ini["Custom Colors"]["yellow"], CC_YELLOW);
			ExtractColorFromString(ini["Custom Colors"]["purple"], CC_PURPLE);
			ExtractColorFromString(ini["Custom Colors"]["cyan"], CC_CYAN);
		}
		if (ini["Custom Colors"].has("outline"))
		{
			ExtractColorFromString(ini["Custom Colors"]["outline"], OUTLINE_CLR);
		}

		/* Log */
		if (ini["Misc"].has("enableLog"))
			LOGFILE_ENABLED = static_cast<bool>(std::atoi(ini["Misc"]["enableLog"].c_str()));
	}
} // namespace util