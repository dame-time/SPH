//
// Created by Davide Paollilo on 02/11/23.
//

#include "../Window.hpp"

#include <stdexcept>
#include <iostream>

int Window::viewportH;
int Window::viewportW;

Window::Window(Camera* mainCamera) : mainCamera(mainCamera) {
	SCR_WIDTH = 640;
	SCR_HEIGHT = 480;
	
	viewportW = SCR_WIDTH;
	viewportH = SCR_HEIGHT;
	
	// Initialize GLFW
	if (!glfwInit())
		throw std::runtime_error("Failed to initialize GLFW.");
	
	// Set GLFW to not create an OpenGL context
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	
#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
	
	window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "OpenGL Renderer", nullptr, nullptr);
	if (!window) {
		glfwTerminate();
		throw std::runtime_error("Failed to create GLFW window.");
	}
	
	glfwMakeContextCurrent(window);
	
	GLFWframebuffersizefun frameBufferSizeCallback = [](GLFWwindow* win, int width, int height)
	{
		viewportH = height;
		viewportW = width;
		
		glViewport(0, 0, width, height);
	};
	
	GLFWcursorposfun mouseCallback = [](GLFWwindow* win, double xPos, double yPos)
	{
		auto windowClassInstance = static_cast<Window*>(glfwGetWindowUserPointer(win));
		
		if(glfwGetKey(win, GLFW_KEY_LEFT_SUPER) == GLFW_PRESS || glfwGetKey(win, GLFW_KEY_RIGHT_SUPER) == GLFW_PRESS) {
			if (!windowClassInstance->commandPressed) {
				windowClassInstance->commandPressed = true;
				windowClassInstance->lastX = xPos;
				windowClassInstance->lastY = yPos;
			}
			
			Math::Scalar xOffset = windowClassInstance->lastX - xPos;
			Math::Scalar yOffset = windowClassInstance->lastY - yPos;
			
			windowClassInstance->lastX = xPos;
			windowClassInstance->lastY = yPos;
			
			Math::Scalar sensitivity = 1;
			
			xOffset *= sensitivity;
			yOffset *= sensitivity;
			
			windowClassInstance->mainCamera->rotateAroundX(yOffset);
			windowClassInstance->mainCamera->rotateAroundY(xOffset);
		} else {
			windowClassInstance->commandPressed = false;
		}
	};
	
	GLFWkeyfun keyCallback = [](GLFWwindow* win, int key, int scancode, int action, int mods)
	{
		auto windowClassInstance = static_cast<Window*>(glfwGetWindowUserPointer(win));
		
		if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
			glfwSetWindowShouldClose(win, true);
		
		if (key == GLFW_KEY_R && action == GLFW_PRESS)
			windowClassInstance->emitter->InitDamBreak(256,
													   windowClassInstance->physicsEngine->GetBounds(),
													   static_cast<int>(windowClassInstance->physicsEngine->GetBounds()
													   .max
													   .coordinates.x / 2),
													   static_cast<int>(windowClassInstance->physicsEngine->GetBounds()
													                                       .max
													                                       .coordinates.y / 2),
													   1.1);
		
		if (key == GLFW_KEY_C && action == GLFW_PRESS)
			windowClassInstance->emitter->Clear();
	};
	
	GLFWmousebuttonfun mouseButtonCallback = [](GLFWwindow* win, int button, int action, int mods) {
		auto windowClassInstance = static_cast<Window*>(glfwGetWindowUserPointer(win));
		if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
			windowClassInstance->isMousePressed = true;
		} else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
			windowClassInstance->isMousePressed = false;
		}
		
		if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
			windowClassInstance->isRightMousePressed = true;
		} else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
			windowClassInstance->isRightMousePressed = false;
		}
	};
	
	glfwSetMouseButtonCallback(window, mouseButtonCallback);
	glfwSetFramebufferSizeCallback(window, frameBufferSizeCallback);
	glfwSetCursorPosCallback(window, mouseCallback);
	glfwSetWindowUserPointer(window, this);
	glfwSetKeyCallback(window, keyCallback);
	
	isMousePressed = false;
	isRightMousePressed = false;
	
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		glfwTerminate();
		throw std::runtime_error("Failed to initialize GLAD");
	}
	
	glEnable(GL_DEPTH_TEST);
	
	wireframeShader = new Renderer::Shader("/Users/davidepaollilo/Workspaces/C++/SPH/Shader/GLSL/wireframe"
										   ".vert",
										   "/Users/davidepaollilo/Workspaces/C++/SPH/Shader/GLSL/wireframe"
										   ".frag");
}

void Window::OnMouseClick(const Math::Vector2& screenPos) {
	// Convert screen coordinates to normalized device coordinates
	double normalizedX = (2.0 * screenPos[0]) / viewportW - 1.0;
	double normalizedY = 1.0 - (2.0 * screenPos[1]) / viewportH;
	
	// Get the inverse of the camera projection matrix to convert to world coordinates
	Math::Matrix4 inverseProj = mainCamera->getOrthographicMatrix(viewportW, viewportH, 0.1, 1000.0).inverse();
	Math::Matrix4 inverseView = mainCamera->getViewMatrix().inverse();
	
	// Transform from NDC to world coordinates
	Math::Vector4 ndcCoords(normalizedX, normalizedY, 0.0, 1.0);
	Math::Vector4 worldCoords = inverseProj * ndcCoords;
	worldCoords = inverseView * worldCoords;
	
	// Normalize by the W component
	worldCoords[0] /= worldCoords[3];
	worldCoords[1] /= worldCoords[3];
	worldCoords[2] /= worldCoords[3];
	
	// Call the ApplyRepulsiveForce function in PhysicsEngine
	Math::Vector2 worldPosition(worldCoords[0], worldCoords[1]);
	double radius = 5.0;   // Example radius
	double strength = 5.0;  // Example force strength
	
	physicsEngine->ApplyRepulsiveForce(worldPosition, radius, strength);
}

void Window::OnRightMouseClick(const Math::Vector2& screenPos) {
	// Convert screen coordinates to normalized device coordinates
	double normalizedX = (2.0 * screenPos[0]) / viewportW - 1.0;
	double normalizedY = 1.0 - (2.0 * screenPos[1]) / viewportH;
	
	// Get the inverse of the camera projection matrix to convert to world coordinates
	Math::Matrix4 inverseProj = mainCamera->getOrthographicMatrix(viewportW, viewportH, 0.1, 1000.0).inverse();
	Math::Matrix4 inverseView = mainCamera->getViewMatrix().inverse();
	
	// Transform from NDC to world coordinates
	Math::Vector4 ndcCoords(normalizedX, normalizedY, 0.0, 1.0);
	Math::Vector4 worldCoords = inverseProj * ndcCoords;
	worldCoords = inverseView * worldCoords;
	
	// Normalize by the W component
	worldCoords[0] /= worldCoords[3];
	worldCoords[1] /= worldCoords[3];
	worldCoords[2] /= worldCoords[3];
	
	// Call the ApplyRepulsiveForce function in PhysicsEngine
	Math::Vector2 worldPosition(worldCoords[0], worldCoords[1]);
	double radius = 5.0;   // Example radius
	double strength = 5.0;  // Example force strength
	
	physicsEngine->ApplyAttractiveForce(worldPosition, radius, strength);
}

Window::~Window() {
	glfwTerminate();
}

void Window::SetParticleEmitter(ParticleEmitter* sg)
{
	this->emitter = sg;
}

void Window::SetPhysicsEngine(PhysicsEngine* pe)
{
	this->physicsEngine = pe;
}

void Window::SetParticleShader(Renderer::Shader* shader)
{
	particleShader = shader;
}

void Window::SetEmitterShader(Renderer::Shader* shader)
{
	emitterShader = shader;
}

void Window::ProcessInput ()
{
	Math::Scalar cameraSpeed = 30 * deltaTime;
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
			mainCamera->scale(-cameraSpeed * deltaTime);
	
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
			mainCamera->scale(cameraSpeed * deltaTime);
}

void Window::Render() {
	mainCamera->translate(40);
	mainCamera->scale(15);
	mainCamera->setTarget(Math::Vector3());
	
	while (!glfwWindowShouldClose(window)) {
		auto perspective = mainCamera->getOrthographicMatrix(viewportW, viewportH, 0.1, 1000.0);
		
		Math::Scalar currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		
		ProcessInput();
		
		if (isMousePressed)
		{
			double xPos, yPos;
			glfwGetCursorPos(this->window, &xPos, &yPos);
			
			OnMouseClick(Math::Vector2(xPos, yPos));
		}
		
		if (isRightMousePressed)
		{
			double xPos, yPos;
			glfwGetCursorPos(this->window, &xPos, &yPos);
			
			OnRightMouseClick(Math::Vector2(xPos, yPos));
		}
		
		glClearColor(0.3f, 0.3f, 0.8f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		// TODO: Check this
//		emitterShader->use();
//		emitterShader->setMat4("view", mainCamera->getViewMatrix());
//		emitterShader->setMat4("projection", perspective);
//
//		emitterShader->setVec3("light.position", Math::Vector3(-1, 1, 0));
//		emitterShader->setVec3("light.ambient", Math::Vector3(.5, .5, .5));
//		emitterShader->setVec3("light.diffuse", Math::Vector3(0.3, 0.3, 0.3));
//		emitterShader->setVec3("light.specular", Math::Vector3(0.3, 0.3, 0.3));
		
		wireframeShader->use();
		wireframeShader->setMat4("model", Math::Matrix4());
		wireframeShader->setMat4("view", mainCamera->getViewMatrix());
		wireframeShader->setMat4("projection", perspective);
		wireframeShader->setVec3("objectColor", Math::Vector3(1, 1, 1));
		
		physicsEngine->RenderWireframeBox();
		
		particleShader->use();
		particleShader->setMat4("view", mainCamera->getViewMatrix());
		particleShader->setMat4("projection", perspective);
		
		particleShader->setVec3("light.position", Math::Vector3(-1, 1, 0));
		particleShader->setVec3("light.ambient", Math::Vector3(.5, .5, .5));
		particleShader->setVec3("light.diffuse", Math::Vector3(0.3, 0.3, 0.3));
		particleShader->setVec3("light.specular", Math::Vector3(0.3, 0.3, 0.3));
		
		for (const auto& particle : emitter->GetParticles())
		{
//			std::cout << "Rendering with radius: " << particle->radius << " at pos: " << std::endl;
//			particle->position.print();
			particle->RenderBillboard(particleShader);
		}
		
		physicsEngine->Update();
		
		glfwSwapBuffers(window);
		glfwPollEvents();
//		glGetError();
	}
}
