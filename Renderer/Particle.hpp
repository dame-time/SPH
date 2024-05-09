//
// Created by Davide Paollilo on 03/11/23.
//

#ifndef SPH_PARTICLE_HPP
#define SPH_PARTICLE_HPP

#include <Vector3.hpp>
#include <Shader.hpp>

class Particle
{
	public:
		Math::Vector3 position;
		Math::Vector3 velocity;
		Math::Vector3 force;
		Math::Scalar mass;
		Math::Scalar radius;
		Math::Scalar density;
		Math::Scalar pressure;
		Math::Scalar viscosity;
		Math::Vector3 color;
		Math::Vector2 gradientSmoothedColorField;
		Math::Scalar laplacianSmoothedColorField;
		Math::Scalar curvature;
	
		Particle(const Particle& particle);
		Particle();
		~Particle();
		
		void Render(const Renderer::Shader* shader) const;
		void RenderBillboard(const Renderer::Shader* shader) const;
};

#endif //SPH_PARTICLE_HPP
