//
// Created by Davide Paollilo on 08/05/24.
//

#include "SpatialGrid.hpp"

GridCoord SpatialGrid::GetCellIndex (const Math::Vector3 &position) const
{
	int x = static_cast<int>(std::floor(position[0] / cellSize));
	int y = static_cast<int>(std::floor(position[1] / cellSize));
	return std::make_tuple(x, y);
}

void SpatialGrid::Clear ()
{
	grid.clear();
}

void SpatialGrid::Insert (Particle* particle)
{
	GridCoord index = GetCellIndex(particle->position);
	grid[index].push_back(particle);
}

std::vector<GridCoord> SpatialGrid::GetNeighborCells (const GridCoord &center)
{
	std::vector<GridCoord> neighbors;
	for (int dx = -1; dx <= 1; ++dx)
		for (int dy = -1; dy <= 1; ++dy)
			neighbors.emplace_back(std::get<0>(center) + dx, std::get<1>(center) + dy);
	
	return neighbors;
}

const std::vector<Particle *> &SpatialGrid::GetCellParticles (const GridCoord &index) const
{
	static const std::vector<Particle*> empty;
	auto it = grid.find(index);
	return it != grid.end() ? it->second : empty;
}
