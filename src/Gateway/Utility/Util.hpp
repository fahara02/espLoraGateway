#ifndef UTIL_HPP
#define UTIL_HPP
#include "GlobalDefines.hpp"
class Util
{
  public:
	template<ChipModel Model>
	static constexpr Series GetSeries()
	{
		switch(Model)
		{
			case ChipModel::RFM95:
				return Series::RFM;
			case ChipModel::SX1272:
				return Series::SM72;
			case ChipModel::SX1273:
				return Series::SM72;
			case ChipModel::SX1276:
				return Series::SM76;
			case ChipModel::SX1277:
				return Series::SM76;
			case ChipModel::SX1278:
				return Series::SM76;
			case ChipModel::SX1279:
				return Series::SM76;
			case ChipModel::SX1261:
				return Series::SM62;
			case ChipModel::SX1262:
				return Series::SM62;
			default:
				return Series::None;
		}
	}

	class ToString
	{
	};
};
#endif