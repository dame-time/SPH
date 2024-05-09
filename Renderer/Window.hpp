//
// Created by Davide Paollilo on 02/11/23.
//

#ifndef SPH_WINDOW_HPP
#define SPH_WINDOW_HPP

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Camera.hpp"
#include "Shader.hpp"
#include "ParticleEmitter.hpp"
#include "PhysicsEngine.hpp"

class Window {
	private:
		unsigned int SCR_WIDTH;
		unsigned int SCR_HEIGHT;
	
		Math::Scalar deltaTime;
		Math::Scalar lastFrame;
	
		bool commandPressed;
		bool isMousePressed;
		bool isRightMousePressed;
		float lastX, lastY;
	
		Renderer::Shader* particleShader;
		Renderer::Shader* emitterShader;
		
		Renderer::Shader* wireframeShader;
		
		GLFWwindow* window;
		Camera* mainCamera;
		ParticleEmitter* emitter;
		
		PhysicsEngine* physicsEngine;
		
		void ProcessInput();
	
		void OnMouseClick(const Math::Vector2& screenPos);
		void OnRightMouseClick(const Math::Vector2& screenPos);
		
	public:
		static int viewportH;
		static int viewportW;
	
		explicit Window(Camera* mainCamera);
		~Window();
		
		void SetParticleEmitter(ParticleEmitter* sg);
		void SetPhysicsEngine(PhysicsEngine* pe);
		
		void SetParticleShader(Renderer::Shader* shader);
		void SetEmitterShader(Renderer::Shader* shader);
		
		void Render();
};

#endif //SPH_WINDOW_HPP
