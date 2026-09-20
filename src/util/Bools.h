#pragma once
#include "Config.h"

namespace util
{
	inline bool CheckBMX(const Config& cfg, const CPed *player)
	{
		if (cfg.ENABLE_BMX || !player || !player->m_pVehicle)
			return false;

		return player->m_pVehicle->m_nVehicleSubClass == VEHICLE_BMX;
	}

	inline bool NavEnabled(const Config& cfg, const CPed *player)
	{
		if (!player || CTheScripts::bMiniGameInProgress)
			return false;

		if (player->m_nPedFlags.bInVehicle && player->m_pVehicle)
		{
			if (player->m_pVehicle->m_nVehicleSubClass == VEHICLE_PLANE ||
				player->m_pVehicle->m_nVehicleSubClass == VEHICLE_HELI)
				return false;

			if (CheckBMX(cfg, player))
				return false;

			return true;
		}

		return cfg.ENABLE_ON_FOOT;
	}
} // namespace util