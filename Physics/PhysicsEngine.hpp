// PhysicsEngine.hpp
#ifndef SPH_PHYSICS_ENGINE_HPP
#define SPH_PHYSICS_ENGINE_HPP

#include "Particle.hpp"
#include "ParticleEmitter.hpp"
#include "BoundingBox.hpp"
#include "SpatialGrid.hpp"

#include <vector>
#include <memory>

class PhysicsEngine
{
	private:
		ParticleEmitter* particleEmitter;
		BoundingBox bounds;
		
		SpatialGrid* spatialGrid{};
	
		Math::Scalar boundsDamping;
	
		const Math::Scalar H = 1.125;
		const Math::Scalar HSQ = H * H;
		
		const Math::Scalar POLY6 = 315.0 / (64.0 * M_PI * std::pow(H, 9.0));
		const Math::Scalar SPIKY_GRAD = -45.0 / (M_PI * std::pow(H, 6.0));
		const Math::Scalar VISC_LAP = 45.0 / (M_PI * std::pow(H, 6.0));
		
		const Math::Scalar REST_DENS = 2.25;
		const Math::Scalar GAS_CONST = 3000.0;
	
		const Math::Vector3 G = Math::Vector3(0, -10.81, 0);
		const Math::Scalar SURFACE_TENSION_COEFFICIENT = 0.07;
		
		void ComputeDensityPressureOverParticles();
		void ComputeSurfaceTensionOverParticles();
		void ComputeForcesOverParticles();
		
		[[nodiscard]] Math::Vector3 GradientSpikyKernel(const Math::Vector3& r_ij) const;
		[[nodiscard]] Math::Scalar LaplacianPoly6Kernel(Math::Scalar r2) const;
		[[nodiscard]] Math::Scalar Poly6Kernel(Math::Scalar r2) const;
		[[nodiscard]] Math::Scalar ViscosityLaplacianKernel(Math::Scalar r) const;
	
	public:
		explicit PhysicsEngine(ParticleEmitter* particleEmitter);
		~PhysicsEngine();
	
	[[maybe_unused]] void SetBoundsDamping(const Math::Scalar& damping);
		
		void Update ();
		
		void ApplyRepulsiveForce(const Math::Vector3& center, Math::Scalar radius, Math::Scalar strength);
		void ApplyAttractiveForce(const Math::Vector3& center, Math::Scalar radius, Math::Scalar strength);
		
		BoundingBox GetBounds()
		{
			return bounds;
		}
	
		void RenderWireframeBox() const;
};

#endif //SPH_PHYSICS_ENGINE_HPP
