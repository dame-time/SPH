#include "Renderer/Window.hpp"
#include "PhysicsEngine.hpp"

int main() {
	auto camera = new Camera();
	Window window = Window(camera);
	auto particleEmitter = new ParticleEmitter();
	auto physicsEngine = new PhysicsEngine(particleEmitter);
	
	[[maybe_unused]] auto particleSolidShader = new Renderer::Shader
			("/Users/davidepaollilo/Workspaces/C++/SPH/Shader/GLSL/sphere.vert",
			 "/Users/davidepaollilo/Workspaces/C++/SPH/Shader/GLSL/sphere.frag");
	auto particleBillboardShader = new Renderer::Shader
			("/Users/davidepaollilo/Workspaces/C++/SPH/Shader/GLSL/impostor.vert",
			 "/Users/davidepaollilo/Workspaces/C++/SPH/Shader/GLSL/impostor.frag");
	
	auto emitterShader = new Renderer::Shader
			("/Users/davidepaollilo/Workspaces/C++/SPH/Shader/GLSL/vertex.vert",
			 "/Users/davidepaollilo/Workspaces/C++/SPH/Shader/GLSL/fragment.frag");
	
	window.SetParticleShader(particleBillboardShader);
	window.SetEmitterShader(emitterShader);
	window.SetParticleEmitter(particleEmitter);
	window.SetPhysicsEngine(physicsEngine);
 
	window.Render();
	
	delete camera;
	
    return 0;
}
