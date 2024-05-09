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
	
		const Math::Scalar H = 1.2;		    // kernel radius
		const Math::Scalar HSQ = H * H;	    // radius^2 for optimization
		
		// smoothing kernels defined in Müller and their gradients
		// adapted to 2D per "SPH Based Shallow Water Simulation" by Solenthaler et al.
		const Math::Scalar POLY6 = 4. / (M_PI * std::pow(H, 8.));
		
		const Math::Scalar REST_DENS = 6.25;  // rest density
		const Math::Scalar GAS_CONST = 2000.; // const for equation of state
	
		const Math::Scalar SPIKY_GRAD = -10. / (M_PI * std::pow(H, 5.));
		const Math::Scalar VISC_LAP = 40. / (M_PI * std::pow(H, 5.));
		
		const Math::Vector3 G = Math::Vector3(0, -10.81, 0);
		const Math::Scalar SURFACE_TENSION_COEFFICIENT = 0.07;
		
		void ComputeDensityPressureOverParticles();
		void ComputeSurfaceTensionOverParticles();
		void ComputeForcesOverParticles();
		
		[[nodiscard]] Math::Vector2 GradientSpikyKernel(const Math::Vector2& r_ij) const;
		[[nodiscard]] Math::Scalar LaplacianPoly6Kernel(Math::Scalar r2) const;
	
	public:
		explicit PhysicsEngine(ParticleEmitter* particleEmitter);
		~PhysicsEngine();
	
	[[maybe_unused]] void SetBoundsDamping(const Math::Scalar& damping);
		
		void Update ();
		
		void ApplyRepulsiveForce(const Math::Vector2& center, Math::Scalar radius, Math::Scalar strength);
		void ApplyAttractiveForce(const Math::Vector2& center, Math::Scalar radius, Math::Scalar strength);
		
		BoundingBox GetBounds()
		{
			return bounds;
		}
	
		void RenderWireframeBox() const;
};

#endif //SPH_PHYSICS_ENGINE_HPP
