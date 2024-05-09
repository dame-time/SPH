#ifndef SPH_SCENE_GRAPH_HPP
#define SPH_SCENE_GRAPH_HPP

#include <vector>
#include <memory>
#include "Particle.hpp"
#include "BoundingBox.hpp"

class ParticleEmitter {
private:
	std::vector<std::shared_ptr<Particle>> particles;

public:
	ParticleEmitter() = default;
	~ParticleEmitter() = default;
	
	void Init(int numParticles);
	void InitFromSampleParticle(int numParticles, const Particle& particle);
	
	void InitDamBreak(int numParticles, const BoundingBox& bounds, const Math::Scalar& paddingX = 1.0,
	                  const Math::Scalar& paddingY = 1.0, const Math::Scalar& paddingZ = 1.0, const Math::Scalar& spacing = 1.0);
	
	void AddParticle(const Particle& particle);
	
	[[nodiscard]] const std::vector<std::shared_ptr<Particle>>& GetParticles() const;
	[[nodiscard]] std::shared_ptr<Particle> GetParticle(size_t index) const;
	void RemoveParticle(size_t index);
	
	void Clear();
};

#endif //SPH_SCENE_GRAPH_HPP