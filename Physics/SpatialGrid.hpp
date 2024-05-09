#include "Particle.hpp"

#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>
#include <functional>

// Type alias for grid coordinates
using GridCoord = std::tuple<int, int>;

struct GridCoordHash {
	std::size_t operator()(const GridCoord& coord) const {
		std::size_t hash1 = std::hash<int>()(std::get<0>(coord));
		std::size_t hash2 = std::hash<int>()(std::get<1>(coord));
		return hash1 ^ (hash2 << 1);  // Combine the two hash values
	}
};

// Grid structure
class SpatialGrid {
public:
	// Cell size should be the same as the smoothing length (h)
	Math::Scalar cellSize;
	
	// Grid map: each cell holds a vector of particle pointers
	std::unordered_map<GridCoord, std::vector<Particle*>, GridCoordHash> grid;
	
	explicit SpatialGrid(Math::Scalar cellSize) : cellSize(cellSize) {}
	
	// Convert particle position to a cell index
	[[nodiscard]] GridCoord GetCellIndex(const Math::Vector3& position) const;
	
	// Clear the grid
	void Clear();
	
	// Insert a particle into the appropriate cell
	void Insert(Particle* particle);
	
	// Get a list of neighboring cells, including the current cell
	static std::vector<GridCoord> GetNeighborCells(const GridCoord& center);
	
	// Retrieve particles from a specific cell
	[[nodiscard]] const std::vector<Particle*>& GetCellParticles(const GridCoord& index) const;
};

