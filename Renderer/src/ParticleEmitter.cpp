//
// Created by Davide Paollilo on 03/11/23.
//
#include "../ParticleEmitter.hpp"

void ParticleEmitter::AddParticle(const Particle& particle) {
	particles.push_back(std::make_shared<Particle>(particle));
}

const std::vector<std::shared_ptr<Particle>>& ParticleEmitter::GetParticles() const {
	return particles;
}

std::shared_ptr<Particle> ParticleEmitter::GetParticle(size_t index) const {
	if (index < particles.size())
		return particles[index];
	
	throw std::out_of_range("Index is out of range.");
}

void ParticleEmitter::RemoveParticle(size_t index) {
	if (index < particles.size())
		particles.erase(particles.begin() + index);
	else
		throw std::out_of_range("Index is out of range.");
}

void ParticleEmitter::Init (int numParticles)
{
	for (int i = -numParticles; i <= numParticles; i++)
	{
		Particle newParticle = Particle();
		newParticle.position = Math::Vector3(i, 0, 0);
		this->AddParticle(newParticle);
	}
}

void ParticleEmitter::InitFromSampleParticle (int numParticles, const Particle &particle)
{
	for (int i = -numParticles; i <= numParticles; i++)
	{
		Particle newParticle = Particle(particle);
		newParticle.position = Math::Vector3(i, 0, 0);
		this->AddParticle(newParticle);
	}
}

void ParticleEmitter::InitDamBreak(int numParticles, const BoundingBox& bounds, const Math::Scalar& paddingX,
                                   const Math::Scalar& paddingY, const Math::Scalar& paddingZ, const Math::Scalar& spacing) {
	Math::Scalar startX = bounds.min.coordinates.x + paddingX;
	Math::Scalar startY = bounds.min.coordinates.y + paddingY;
	Math::Scalar startZ = bounds.min.coordinates.z + paddingZ;
	
	Math::Scalar endX = bounds.max.coordinates.x - paddingX;
	Math::Scalar endY = bounds.max.coordinates.y - paddingY;
	Math::Scalar endZ = bounds.max.coordinates.z - paddingZ;
	
	int particlesX = static_cast<int>((endX - startX) / spacing);
	int particlesY = static_cast<int>((endY - startY) / spacing);
	int particlesZ = static_cast<int>((endZ - startZ) / spacing);
	
	int totalParticles = std::min(numParticles, particlesX * particlesY * particlesZ);
	
	startX += (endX - startX - particlesX * spacing) / 2;
	startY += (endY - startY - particlesY * spacing) / 2;
	startZ += (endZ - startZ - particlesZ * spacing) / 2;
	
	for (int i = 0; i < particlesX; ++i) {
		for (int j = 0; j < particlesY; ++j) {
			for (int k = 0; k < particlesZ; ++k) {
				if (totalParticles <= 0) break;
				
				Particle newParticle = Particle();
				newParticle.position = Math::Vector3(startX + i * spacing, startY + j * spacing, startZ + k * spacing);
				this->AddParticle(newParticle);
				
				--totalParticles;
			}
			
			if (totalParticles <= 0) break;
		}
		
		if (totalParticles <= 0) break;
	}
}

void ParticleEmitter::Clear()
{
	particles.clear();
}
