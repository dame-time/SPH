// PhysicsEngine.cpp
#include "../PhysicsEngine.hpp"

PhysicsEngine::PhysicsEngine(ParticleEmitter* pe) {
	bounds.min = Math::Vector3(-20, -20, -10);
	bounds.max = Math::Vector3(20, 20, 10);
	
	particleEmitter = pe;
	
	boundsDamping = -0.05;
	
	spatialGrid = new SpatialGrid(H);
}

PhysicsEngine::~PhysicsEngine() = default;

void PhysicsEngine::Update() {
	const static Math::Scalar DT = 0.015;
	const static Math::Scalar restitution = 0.85;
	
	ComputeDensityPressureOverParticles();
	ComputeSurfaceTensionOverParticles();
	ComputeForcesOverParticles();
	
	for (auto& particle : particleEmitter->GetParticles()) {
		particle->velocity += DT * particle->force / particle->density;
		particle->position += DT * particle->velocity;
		
		for (short i = 0; i < 3; ++i) {
			if (particle->position[i] < bounds.min[i]) {
				particle->position[i] = bounds.min[i];
				particle->velocity[i] *= -restitution;
			}
			
			if (particle->position[i] > bounds.max[i]) {
				particle->position[i] = bounds.max[i];
				particle->velocity[i] *= -restitution;
			}
		}
	}
}

void PhysicsEngine::ComputeDensityPressureOverParticles() {
	auto particles = particleEmitter->GetParticles();
	spatialGrid->Clear();
	
	for (auto& particle : particles) {
		spatialGrid->Insert(particle.get());
	}
	
	for (auto& particle : particles) {
		GridCoord cellIndex = spatialGrid->GetCellIndex(particle->position);
		std::vector<Particle*> neighbors;
		
		for (const auto& neighborCell : SpatialGrid::GetNeighborCells(cellIndex)) {
			const auto& cellParticles = spatialGrid->GetCellParticles(neighborCell);
			neighbors.insert(neighbors.end(), cellParticles.begin(), cellParticles.end());
		}
		
		particle->density = 0.0;
		for (const auto& neighbor : neighbors) {
			Math::Vector3 r_ij = (particle->position - neighbor->position);
			
			Math::Scalar r_ij_sq = r_ij.squareMagnitude();
			
			if (r_ij_sq <= HSQ && r_ij_sq >= 0.0) {
				particle->density += neighbor->mass * Poly6Kernel(r_ij_sq);
			}
		}
		particle->pressure = GAS_CONST * (particle->density - REST_DENS);
	}
}

void PhysicsEngine::ComputeForcesOverParticles() {
	auto particles = particleEmitter->GetParticles();
	spatialGrid->Clear();
	
	for (auto& particle : particles) {
		spatialGrid->Insert(particle.get());
	}
	
	for (auto& pi : particles) {
		auto pressure = Math::Vector3();
		auto viscosity = Math::Vector3();
		auto surfaceTension = Math::Vector3();
		
		GridCoord cellIndex = spatialGrid->GetCellIndex(pi->position);
		std::vector<Particle*> neighbors;
		
		for (const auto& neighborCell : spatialGrid->GetNeighborCells(cellIndex)) {
			const auto& cellParticles = spatialGrid->GetCellParticles(neighborCell);
			neighbors.insert(neighbors.end(), cellParticles.begin(), cellParticles.end());
		}
		
		for (const auto& pj : neighbors) {
			if (pi.get() == pj) continue;

			Math::Vector3 r_ij = pj->position - pi->position;
			Math::Scalar r_ij_mag = r_ij.magnitude();
			
			auto spikyGrad = GradientSpikyKernel(r_ij);

			if (r_ij_mag <= H && r_ij_mag >= 0) {
				pressure += spikyGrad * pj->mass * (pi->pressure + pj->pressure) / (2.0 * pj->density);;
				
				viscosity += pj->viscosity * pj->mass * (pj->velocity - pi->velocity) / pj->density *
						ViscosityLaplacianKernel(r_ij_mag);
				
				bool shouldEvaluateSurfaceTension = pj->gradientSmoothedColorField.magnitude() > 0.05;
				Math::Vector3 normalized_n_i = pj->gradientSmoothedColorField.normalized();
				Math::Scalar k = SURFACE_TENSION_COEFFICIENT * pj->curvature;

				if (shouldEvaluateSurfaceTension)
					surfaceTension += (normalized_n_i * k);
			}
		}
		
		auto gravity = G * pi->mass / pi->density;
		pi->force = pressure + viscosity + gravity + surfaceTension;
	}
}

void PhysicsEngine::ComputeSurfaceTensionOverParticles() {
	auto particles = particleEmitter->GetParticles();
	spatialGrid->Clear();
	
	for (auto &particle: particles)
		spatialGrid->Insert(particle.get());
	
	for (auto& pi : particles) {
		pi->gradientSmoothedColorField = Math::Vector3();
		pi->laplacianSmoothedColorField = 0.0;
		
		GridCoord cellIndex = spatialGrid->GetCellIndex(pi->position);
		std::vector<Particle*> neighbors;
		
		for (const auto& neighborCell : SpatialGrid::GetNeighborCells(cellIndex)) {
			const auto& cellParticles = spatialGrid->GetCellParticles(neighborCell);
			neighbors.insert(neighbors.end(), cellParticles.begin(), cellParticles.end());
		}
		
		for (const auto& pj : neighbors) {
			if (pi.get() == pj) continue;
			
			Math::Vector3 r_ij = (pi->position - pj->position);
			Math::Scalar r_ij_sq = r_ij.squareMagnitude();

			Math::Vector3 grad_W_ij = GradientSpikyKernel(r_ij);
			
			Math::Scalar constant = pj->mass / pj->density;
			pi->gradientSmoothedColorField += grad_W_ij * constant;
			
			Math::Scalar lap_W_ij = LaplacianPoly6Kernel(r_ij_sq);
			pi->laplacianSmoothedColorField += constant * lap_W_ij;
		}
		
		Math::Scalar normal_magnitude = pi->gradientSmoothedColorField.magnitude();
		pi->curvature = -pi->laplacianSmoothedColorField / (normal_magnitude != 0.0 ? normal_magnitude : 1.0);
	}
}

Math::Scalar PhysicsEngine::Poly6Kernel(Math::Scalar r2) const {
	if (r2 <= HSQ && r2 >= 0.0)
	{
		Math::Scalar diff = HSQ - r2;
		return POLY6 * std::pow(diff, 3);
	} else
		return 0.0;
}

Math::Scalar PhysicsEngine::ViscosityLaplacianKernel(Math::Scalar r) const {
	if (r > 0.0 && r < H)
		return VISC_LAP * (H - r);
	else
		return 0.0;
}

Math::Vector3 PhysicsEngine::GradientSpikyKernel(const Math::Vector3& r_ij) const {
	auto r = r_ij.magnitude();
	
	if (r > 0.0 && r < H)
		return r_ij.normalized() * SPIKY_GRAD * std::pow(H - r, 2);
	else
		return {0.0, 0.0, 0.0};
}

Math::Scalar PhysicsEngine::LaplacianPoly6Kernel(Math::Scalar r2) const {
	// Math::Scalar derivative of Poly6 kernel (adjusted for 3D)
	const Math::Scalar POLY6_LAPLACIAN_COEFF = 315.0 / (64 * M_PI * std::pow(H, 9));
	
	if (r2 <= HSQ && r2 >= 0.0)
	{
		Math::Scalar diff = HSQ - r2;
		return POLY6_LAPLACIAN_COEFF * diff * (r2 - 3 * HSQ);
	}
	else
		return 0.0;
}

void PhysicsEngine::ApplyRepulsiveForce(const Math::Vector3& center, Math::Scalar radius, Math::Scalar strength) {
	auto particles = particleEmitter->GetParticles();
	
	for (auto& particle : particles) {
		Math::Vector3 displacement = particle->position - center;
		Math::Scalar distance = displacement.magnitude();
		
		if (distance < radius && distance > 0) {
			Math::Vector3 outwardDirection = displacement / distance;
			Math::Scalar attenuation = (radius - distance) / radius;
			Math::Scalar repulsiveForce = strength * attenuation;
			
			particle->velocity += outwardDirection * repulsiveForce;
		}
	}
}

void PhysicsEngine::ApplyAttractiveForce(const Math::Vector3& center, Math::Scalar radius, Math::Scalar strength) {
	auto separationDistance = 1.25;
	auto particles = particleEmitter->GetParticles();
	
	for (auto& particle : particles) {
		Math::Vector3 displacement = center - particle->position;
		Math::Scalar distanceToCenter = displacement.magnitude();
		
		if (distanceToCenter < radius && distanceToCenter > 0) {
			Math::Vector3 inwardDirection = displacement / distanceToCenter;
			Math::Scalar attenuation = (radius - distanceToCenter) / radius;
			Math::Scalar attractiveForce = strength * attenuation;
			
			particle->velocity += inwardDirection * attractiveForce;
		}
		
		// Apply separation force to prevent clustering
		for (auto& otherParticle : particles) {
			if (particle == otherParticle) continue;
			
			Math::Vector3 displacementBetween = particle->position - otherParticle->position;
			Math::Scalar distanceBetween = displacementBetween.magnitude();
			
			// If particles are too close, apply separation force
			if (distanceBetween > 0 && distanceBetween < separationDistance) {
				Math::Vector3 separationDirection = displacementBetween / distanceBetween;
				Math::Scalar separationForce = (separationDistance - distanceBetween) / separationDistance * strength;
				
				particle->velocity += separationDirection * separationForce;
			}
		}
	}
}

void PhysicsEngine::RenderWireframeBox() const {
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
	
	GLuint VAO, VBO, EBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);
	
	glBindVertexArray(VAO);
	
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)nullptr);
	glEnableVertexAttribArray(0);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	
	glBindVertexArray(VAO);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDrawElements(GL_LINES, 24, GL_UNSIGNED_INT, nullptr);
	glBindVertexArray(0);
	
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
	
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
