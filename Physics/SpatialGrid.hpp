#include "Particle.hpp"

#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>
#include <functional>

using GridCoord = std::tuple<int, int, int>;

struct GridCoordHash {
	std::size_t operator()(const GridCoord& coord) const {
		std::size_t hash1 = std::hash<int>()(std::get<0>(coord));
		std::size_t hash2 = std::hash<int>()(std::get<1>(coord));
		std::size_t hash3 = std::hash<int>()(std::get<2>(coord));
		return hash1 ^ (hash2 << 1) ^ (hash3 << 2);
	}
};

class SpatialGrid {
public:
	Math::Scalar cellSize;
	std::unordered_map<GridCoord, std::vector<Particle*>, GridCoordHash> grid;
	
	explicit SpatialGrid(Math::Scalar cellSize) : cellSize(cellSize) {}
	
	[[nodiscard]] GridCoord GetCellIndex(const Math::Vector3& position) const;
	void Clear();
	void Insert(Particle* particle);
	static std::vector<GridCoord> GetNeighborCells(const GridCoord& center);
	[[nodiscard]] const std::vector<Particle*>& GetCellParticles(const GridCoord& index) const;
};

