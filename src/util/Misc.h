#pragma once
#include <iomanip>
#include <sstream>
#include <string>
#include "SIMDString.h"

namespace util
{
	// Meters to yards.
	constexpr float mtoyard(float m)
	{
		return m * 1.094f;
	}

	inline SIMDString<64> Float2String(const float in, unsigned char precision = 2)
	{
		std::ostringstream stream;
		stream << std::fixed << std::setprecision(precision) << in;
		return SIMDString<64>(stream.str());
	}

	inline std::string formatDist(float dist, const bool units, bool showETA = false, float speedMs = 0.0f)
	{
		std::ostringstream ss;
		if (!units) // Metric
		{
			if (dist > 999.0f)
				ss << std::fixed << std::setprecision(1) << (dist / 1000.0f) << " KM";
			else
				ss << std::fixed << std::setprecision(0) << dist << " m";
		}
		else // Imperial
		{
			float yrd = mtoyard(dist);
			if (yrd > 599.0f)
				ss << std::fixed << std::setprecision(1) << (yrd / 1760.0f) << " Mi";
			else
				ss << std::fixed << std::setprecision(0) << yrd << " yrds";
		}

		if (showETA && dist > 15.0f)
		{
			float calcSpeed = speedMs;
			if (calcSpeed < 3.0f)
				calcSpeed = 16.0f; // Default cruising speed (~58 km/h)

			int totalSec = static_cast<int>(dist / calcSpeed);
			ss << " (";
			if (totalSec < 60)
			{
				ss << totalSec << "s";
			}
			else if (totalSec < 3600)
			{
				int m = totalSec / 60;
				int s = totalSec % 60;
				ss << m << "m " << s << "s";
			}
			else
			{
				int h = totalSec / 3600;
				int m = (totalSec % 3600) / 60;
				ss << h << "h " << m << "m";
			}
			ss << ")";
		}

		return ss.str();
	}

	inline SIMDString<64> makeDist(float dist, const bool units)
	{
		return SIMDString<64>(formatDist(dist, units, false, 0.0f));
	}
} // namespace util