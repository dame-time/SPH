// PhysicsEngine.cpp
#include "../PhysicsEngine.hpp"

PhysicsEngine::PhysicsEngine(ParticleEmitter* pe) {
	bounds.min = Math::Vector3(-30, -30, -10);
	bounds.max = Math::Vector3(30, 30, 10);
	
	particleEmitter = pe;
	
	boundsDamping = -0.05;
	
	spatialGrid = new SpatialGrid(H);
}

PhysicsEngine::~PhysicsEngine() = default;

void PhysicsEngine::Update() {
	const static Math::Scalar DT = 0.0125;
	const static Math::Scalar restitution = 0.85;
	
	ComputeDensityPressureOverParticles();
	ComputeSurfaceTensionOverParticles();
	ComputeForcesOverParticles();
	
	for (auto& particle : particleEmitter->GetParticles()) {
		// Update velocity and position
		particle->velocity += DT * particle->force / particle->density;
		particle->position += DT * particle->velocity;
		
		// Check collisions for each axis
		for (short i = 0; i < 3; ++i) {
			// Collision with the minimum bound
			if (particle->position[i] < bounds.min[i]) {
				particle->position[i] = bounds.min[i];
				particle->velocity[i] *= -restitution;  // Invert velocity and apply restitution
			}
			
			// Collision with the maximum bound
			if (particle->position[i] > bounds.max[i]) {
				particle->position[i] = bounds.max[i];
				particle->velocity[i] *= -restitution;  // Invert velocity and apply restitution
			}
		}
	}
}

void PhysicsEngine::ComputeDensityPressureOverParticles() {
	auto particles = particleEmitter->GetParticles();
	spatialGrid->Clear();
	
	// Populate the grid with particles
	for (auto& particle : particles) {
		spatialGrid->Insert(particle.get());
	}
	
	// Compute density and pressure
	for (auto& particle : particles) {
		GridCoord cellIndex = spatialGrid->GetCellIndex(particle->position);
		std::vector<Particle*> neighbors;
		
		// Collect neighbors from surrounding cells
		for (const auto& neighborCell : SpatialGrid::GetNeighborCells(cellIndex)) {
			const auto& cellParticles = spatialGrid->GetCellParticles(neighborCell);
			neighbors.insert(neighbors.end(), cellParticles.begin(), cellParticles.end());
		}
		
		// Calculate density and pressure using neighbors
		particle->density = 0.0;
		for (const auto& neighbor : neighbors) {
			Math::Vector3 r_ij_3D = (particle->position - neighbor->position);
			auto r_ij = Math::Vector2(r_ij_3D[0], r_ij_3D[1]);
			
			double r_ij_sq = r_ij.squareMagnitude();
			
			if (r_ij_sq <= HSQ && r_ij_sq >= 0.0) {
				particle->density += neighbor->mass * POLY6 * std::pow(HSQ - r_ij_sq, 3.0);
			}
		}
		particle->pressure = GAS_CONST * (particle->density - REST_DENS);
	}
}

void PhysicsEngine::ComputeForcesOverParticles() {
	auto particles = particleEmitter->GetParticles();
	spatialGrid->Clear();

	// Populate the grid with particles
	for (auto& particle : particles) {
		spatialGrid->Insert(particle.get());
	}

	// Compute forces
	for (auto& pi : particles) {
		auto pressure = Math::Vector3();
		auto viscosity = Math::Vector3();
		auto surfaceTension = Math::Vector3();

		// Get neighboring cells for particle `pi`
		GridCoord cellIndex = spatialGrid->GetCellIndex(pi->position);
		std::vector<Particle*> neighbors;

		// Collect neighbors from surrounding cells
		for (const auto& neighborCell : spatialGrid->GetNeighborCells(cellIndex)) {
			const auto& cellParticles = spatialGrid->GetCellParticles(neighborCell);
			neighbors.insert(neighbors.end(), cellParticles.begin(), cellParticles.end());
		}

		// Calculate forces using neighbors
		for (const auto& pj : neighbors) {
			if (pi.get() == pj) continue;

			Math::Vector3 r_ij = pj->position - pi->position;
			double r_ij_mag = r_ij.magnitude();

			if (r_ij_mag <= H && r_ij_mag >= 0) {
				// Compute pressure force using Spiky kernel
				pressure += -r_ij.normalized() * pj->mass * (pi->pressure + pj->pressure) / (2. * pj->density) *
				            SPIKY_GRAD * std::pow(H - r_ij_mag, 3.);

				// Compute viscosity force
				viscosity += pj->viscosity * pj->mass * (pj->velocity - pi->velocity) / pj->density *
				             VISC_LAP * (H - r_ij_mag);

				// Compute surface tension force
				bool shouldEvaluateSurfaceTension = pj->gradientSmoothedColorField.magnitude() > 0.05;
				Math::Vector2 normalized_n_i = pj->gradientSmoothedColorField.normalized();
				Math::Scalar k = SURFACE_TENSION_COEFFICIENT * pj->curvature;
				Math::Vector2 surfaceTension2D = Math::Vector2(k * normalized_n_i[0], k * normalized_n_i[1]);

				if (shouldEvaluateSurfaceTension)
					surfaceTension += Math::Vector3(surfaceTension2D[0], surfaceTension2D[1], 0);
			}
		}

		// Apply gravity and combine forces
		auto gravity = G * pi->mass / pi->density;
		pi->force = pressure + viscosity + gravity + surfaceTension;
	}
}

void PhysicsEngine::ComputeSurfaceTensionOverParticles() {
	auto particles = particleEmitter->GetParticles();
	spatialGrid->Clear();
	
	// Populate the grid with particles
	for (auto &particle: particles)
		spatialGrid->Insert(particle.get());
	
	
	// Compute surface tension parameters
	for (auto& pi : particles) {
		pi->gradientSmoothedColorField = Math::Vector2();
		pi->laplacianSmoothedColorField = 0.0;
		
		// Get neighboring cells for particle `pi`
		GridCoord cellIndex = spatialGrid->GetCellIndex(pi->position);
		std::vector<Particle*> neighbors;
		
		// Collect neighbors from surrounding cells
		for (const auto& neighborCell : SpatialGrid::GetNeighborCells(cellIndex)) {
			const auto& cellParticles = spatialGrid->GetCellParticles(neighborCell);
			neighbors.insert(neighbors.end(), cellParticles.begin(), cellParticles.end());
		}
		
		// Compute gradient and Laplacian using neighbors
		for (const auto& pj : neighbors) {
			if (pi.get() == pj) continue;  // Skip the same particle
			
			Math::Vector3 r_ij_3D = (pi->position - pj->position);
			auto r_ij = Math::Vector2(r_ij_3D[0], r_ij_3D[1]);
			double r_ij_sq = r_ij.squareMagnitude();
			
			// Compute gradient of Spiky kernel
			Math::Vector2 grad_W_ij = GradientSpikyKernel(r_ij);
			
			// Contribution to gradient field
			Math::Scalar constant = pj->mass / pj->density;
			pi->gradientSmoothedColorField += grad_W_ij * constant;
			
			// Contribution to Laplacian field
			Math::Scalar lap_W_ij = LaplacianPoly6Kernel(r_ij_sq);
			pi->laplacianSmoothedColorField += constant * lap_W_ij;
		}
		
		// Calculate curvature
		Math::Scalar normal_magnitude = pi->gradientSmoothedColorField.magnitude();
		pi->curvature = -pi->laplacianSmoothedColorField / (normal_magnitude != 0.0 ? normal_magnitude : 1.0);
	}
}

Math::Vector2 PhysicsEngine::GradientSpikyKernel(const Math::Vector2& r_ij) const {
	auto r = r_ij.magnitude();
	
	if (r > 0.0 && r < H)
		return r_ij.normalized() * SPIKY_GRAD * std::pow(H - r, 2);
	else
		return {0.0, 0.0};
}

Math::Scalar PhysicsEngine::LaplacianPoly6Kernel(Math::Scalar r2) const {
	// Double derivative of Poly6 kernel
	const double POLY6_LAPLACIAN_COEFF = 24.0 / (M_PI * std::pow(H, 8));
	
	if (r2 <= HSQ && r2 >= 0.0) {
		double diff = HSQ - r2;
		return POLY6_LAPLACIAN_COEFF * diff * (r2 - 3 * HSQ);
	} else {
		return 0.0;
	}
}

void PhysicsEngine::ApplyRepulsiveForce(const Math::Vector2& center, Math::Scalar radius, Math::Scalar strength) {
	auto particles = particleEmitter->GetParticles();
	
	for (auto& particle : particles) {
		// Convert the particle's 3D position to 2D (assume XY plane)
		Math::Vector2 particlePosition2D = Math::Vector2(particle->position[0], particle->position[1]);
		
		// Calculate the displacement vector from the center
		Math::Vector2 displacement = particlePosition2D - center;
		double distance = displacement.magnitude();
		
		// If the particle is within the circle's radius
		if (distance < radius && distance > 0) {
			// Normalize the displacement vector and apply the repulsive force
			Math::Vector2 outwardDirection = displacement / distance;
			double attenuation = (radius - distance) / radius;  // Force attenuation by distance
			double repulsiveForce = strength * attenuation;
			
			// Add the force to the particle's velocity
			particle->velocity[0] += outwardDirection[0] * repulsiveForce;
			particle->velocity[1] += outwardDirection[1] * repulsiveForce;
		}
	}
}

void PhysicsEngine::ApplyAttractiveForce(const Math::Vector2& center, Math::Scalar radius, Math::Scalar strength) {
	auto separationDistance = 1.25;
	auto particles = particleEmitter->GetParticles();
	
	for (auto& particle : particles) {
		// Convert the particle's 3D position to 2D (assume XY plane)
		Math::Vector2 particlePosition2D = Math::Vector2(particle->position[0], particle->position[1]);
		
		// Calculate the displacement vector from the particle to the center
		Math::Vector2 displacement = center - particlePosition2D;
		double distanceToCenter = displacement.magnitude();
		
		// If the particle is within the circle's radius
		if (distanceToCenter < radius && distanceToCenter > 0) {
			// Normalize the displacement vector and apply the attractive force
			Math::Vector2 inwardDirection = displacement / distanceToCenter;
			double attenuation = (radius - distanceToCenter) / radius;
			double attractiveForce = strength * attenuation;
			
			// Add the inward force to the particle's velocity
			particle->velocity[0] += inwardDirection[0] * attractiveForce;
			particle->velocity[1] += inwardDirection[1] * attractiveForce;
		}
		
		// Apply separation force to prevent clustering
		for (auto& otherParticle : particles) {
			if (particle == otherParticle) continue;  // Skip the same particle
			
			// Convert to 2D and calculate displacement between particles
			Math::Vector2 otherPosition2D = Math::Vector2(otherParticle->position[0], otherParticle->position[1]);
			Math::Vector2 displacementBetween = particlePosition2D - otherPosition2D;
			double distanceBetween = displacementBetween.magnitude();
			
			// If particles are too close, apply separation force
			if (distanceBetween > 0 && distanceBetween < separationDistance) {
				Math::Vector2 separationDirection = displacementBetween / distanceBetween;
				double separationForce = (separationDistance - distanceBetween) / separationDistance * strength;
				
				// Apply the separation force
				particle->velocity[0] += separationDirection[0] * separationForce;
				particle->velocity[1] += separationDirection[1] * separationForce;
			}
		}
	}
}

void PhysicsEngine::RenderWireframeBox() const {
	// The 8 vertices of the box
	float vertices[] = {
			static_cast<float>(bounds.min.coordinates.x), static_cast<float>(bounds.min.coordinates.y), static_cast<float>(bounds.min.coordinates.z), // 0: Bottom-left-front
			static_cast<float>(bounds.max.coordinates.x), static_cast<float>(bounds.min.coordinates.y), static_cast<float>(bounds.min.coordinates.z), // 1: Bottom-right-front
			static_cast<float>(bounds.min.coordinates.x), static_cast<float>(bounds.max.coordinates.y), static_cast<float>(bounds.min.coordinates.z), // 2: Top-left-front
			static_cast<float>(bounds.max.coordinates.x), static_cast<float>(bounds.max.coordinates.y), static_cast<float>(bounds.min.coordinates.z), // 3: Top-right-front
			static_cast<float>(bounds.min.coordinates.x), static_cast<float>(bounds.min.coordinates.y), static_cast<float>(bounds.max.coordinates.z), // 4: Bottom-left-back
			static_cast<float>(bounds.max.coordinates.x), static_cast<float>(bounds.min.coordinates.y), static_cast<float>(bounds.max.coordinates.z), // 5: Bottom-right-back
			static_cast<float>(bounds.min.coordinates.x), static_cast<float>(bounds.max.coordinates.y), static_cast<float>(bounds.max.coordinates.z), // 6: Top-left-back
			static_cast<float>(bounds.max.coordinates.x), static_cast<float>(bounds.max.coordinates.y), static_cast<float>(bounds.max.coordinates.z), // 7: Top-right-back
	};
	
	// Indices for the 12 lines of the box
	unsigned int indices[] = {
			0, 1, // Bottom-front edge
			1, 3, // Right-front edge
			3, 2, // Top-front edge
			2, 0, // Left-front edge
			4, 5, // Bottom-back edge
			5, 7, // Right-back edge
			7, 6, // Top-back edge
			6, 4, // Left-back edge
			0, 4, // Bottom-left edge
			1, 5, // Bottom-right edge
			2, 6, // Top-left edge
			3, 7  // Top-right edge
	};
	
	// Create and bind VAO and VBO
	GLuint VAO, VBO, EBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);
	
	glBindVertexArray(VAO);
	
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	
	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)nullptr);
	glEnableVertexAttribArray(0);
	
	// Unbind the VBO
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	
	// Draw the box
	glBindVertexArray(VAO);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // Wireframe mode
	glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, nullptr);
	glBindVertexArray(0); // Unbind VAO
	
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL ); // Back to normal
	
	// Clean up
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &EBO);
}

[[maybe_unused]] void PhysicsEngine::SetBoundsDamping (const Math::Scalar &damping)
{
	if (damping > -0.01)
		this->boundsDamping = -0.01;
	else if (damping < -50)
		this->boundsDamping = -50;
	else
		this->boundsDamping = damping;
}
