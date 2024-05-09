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

void ParticleEmitter::InitDamBreak(int numParticles, const BoundingBox& bounds, const Math::Scalar& paddingX, const
Math::Scalar& paddingY, const Math::Scalar& spacing)
{
	// Adjust the starting X and Y to account for padding
	Math::Scalar startX = bounds.min.coordinates.x + paddingX;
	Math::Scalar startY = bounds.min.coordinates.y + paddingY;
	
	// Calculate the ending positions accounting for padding
	Math::Scalar endX = bounds.max.coordinates.x - paddingX;
	Math::Scalar endY = bounds.max.coordinates.y - paddingY;
	
	// Calculate the number of particles that can fit within the padded area
	int particlesX = static_cast<int>((endX - startX) / spacing);
	int particlesY = static_cast<int>((endY - startY) / spacing);
	
	// Limit the total number of particles to the number specified
	int totalParticles = std::min(numParticles, particlesX * particlesY);
	
	// We may have to adjust the start position if we cannot fit the exact number of particles
	// Redistribute the remaining space after fitting the particles as additional padding
	startX += (endX - startX - particlesX * spacing) / 2;
	startY += (endY - startY - particlesY * spacing) / 2;
	
	for (int i = 0; i < particlesX; ++i)
	{
		for (int j = 0; j < particlesY; ++j)
		{
			if(totalParticles <= 0) break;
			
			Particle newParticle = Particle();
			newParticle.position = Math::Vector3(startX + i * spacing, startY + j * spacing, 0);
			this->AddParticle(newParticle);
			
			--totalParticles;
		}
		
		if(totalParticles <= 0) break; // This will exit the outer loop if we have no more particles to place
	}
}

void ParticleEmitter::Clear()
{
	particles.clear();
}
