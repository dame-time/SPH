//
// Created by Davide Paollilo on 03/11/23.
//

#include "../Particle.hpp"
#include "../Window.hpp"

#include <vector>

Particle::Particle() {
	this->position = Math::Vector3();
	this->radius = 0.5;
	this->color = Math::Vector3(1, 0, 0);
	
	this->velocity = Math::Vector3();
	this->force = Math::Vector3();
	
	this->pressure = 0;
	this->density = 0;
	this->mass = 2;
	this->viscosity = 4.25;
	this->gradientSmoothedColorField = Math::Vector3();
	this->laplacianSmoothedColorField = 0;
	this->curvature = 0;
}

Particle::Particle (const Particle &particle)
{
	this->position = particle.position;
	this->velocity = particle.velocity;
	this->force = particle.force;
	this->mass = particle.mass;
	this->radius = particle.radius;
	this->density = particle.density;
	this->pressure = particle.pressure;
	this->viscosity = particle.viscosity;
	this->color = particle.color;
	this->gradientSmoothedColorField = particle.gradientSmoothedColorField;
	this->laplacianSmoothedColorField = particle.laplacianSmoothedColorField;
	this->curvature = particle.curvature;
}


Particle::~Particle() = default;

void Particle::Render(const Renderer::Shader* shader) const {
	static GLuint VAO = 0xBAD, VBO = 0xBAD , EBO = 0xBAD;
	static std::vector<float> vertices;
	static std::vector<int> faces;
	
	if (VAO == 0xBAD)
	{
		// Create an icosahedron
		float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
		std::vector<float> initialVertices = {
				-1,  t,  0,
				1,  t,  0,
				-1, -t,  0,
				1, -t,  0,
				0, -1,  t,
				0,  1,  t,
				0, -1, -t,
				0,  1, -t,
				t,  0, -1,
				t,  0,  1,
				-t,  0, -1,
				-t,  0,  1,
		};
		
		for (int i = 0; i < initialVertices.size(); i += 3) {
			float length = std::sqrt(initialVertices[i]*initialVertices[i] +
			                         initialVertices[i+1]*initialVertices[i+1] +
			                         initialVertices[i+2]*initialVertices[i+2]);
			vertices.push_back(initialVertices[i] / length);
			vertices.push_back(initialVertices[i+1] / length);
			vertices.push_back(initialVertices[i+2] / length);
		}
		
		faces = {
				0, 11, 5,
				0, 5, 1,
				0, 1, 7,
				0, 7, 10,
				0, 10, 11,
				1, 5, 9,
				5, 11, 4,
				11, 10, 2,
				10, 7, 6,
				7, 1, 8,
				3, 9, 4,
				3, 4, 2,
				3, 2, 6,
				3, 6, 8,
				3, 8, 9,
				4, 9, 5,
				2, 4, 11,
				6, 2, 10,
				8, 6, 7,
				9, 8, 1
		};
		
		int subdivisions = 2;
		std::vector<int> newFaces;
		for (int level = 0; level < subdivisions; level++) {
			newFaces.clear();
			for (int i = 0; i < faces.size(); i += 3) {
				int mid[3];
				for (int edge = 0; edge < 3; edge++) {
					int i1 = faces[i + edge];
					int i2 = faces[i + (edge + 1) % 3];
					float midpoint[3] = {
							(vertices[i1 * 3 + 0] + vertices[i2 * 3 + 0]) / 2,
							(vertices[i1 * 3 + 1] + vertices[i2 * 3 + 1]) / 2,
							(vertices[i1 * 3 + 2] + vertices[i2 * 3 + 2]) / 2,
					};
					float length = std::sqrt(midpoint[0] * midpoint[0] + midpoint[1] * midpoint[1] + midpoint[2] * midpoint[2]);
					midpoint[0] /= length;
					midpoint[1] /= length;
					midpoint[2] /= length;
					vertices.insert(vertices.end(), midpoint, midpoint + 3);
					mid[edge] = (int)vertices.size() / 3 - 1;
				}
				newFaces.insert(newFaces.end(), {
						faces[i], mid[0], mid[2],
						faces[i + 1], mid[1], mid[0],
						faces[i + 2], mid[2], mid[1],
						mid[0], mid[1], mid[2]
				});
			}
			
			faces.swap(newFaces);
		}
		
		glGenVertexArrays(1, &VAO);
		glBindVertexArray(VAO);
		
		glGenBuffers(1, &VBO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
		
		// position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		
		glGenBuffers(1, &EBO);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, faces.size() * sizeof(unsigned int), faces.data(), GL_STATIC_DRAW);
		
		glBindVertexArray(0);
	}
	
	shader->use();
	shader->setVec3("center", position);
	shader->setVec3("material.ambient", color);
	shader->setVec3("material.diffuse", Math::Vector3(0.9, 0.9, 0.9));
	shader->setVec3("material.specular", Math::Vector3(0, 0, 0));
	shader->setFloat("material.shininess", 0);
	shader->setFloat("radius", radius);
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//        glEnable(GL_BLEND);
//        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, (int)faces.size(), GL_UNSIGNED_INT, nullptr);
	glBindVertexArray(0);
	glUseProgram(0);
//        glDisable(GL_BLEND);
}

void Particle::RenderBillboard (const Renderer::Shader *shader) const
{
	static GLuint VAO = 0xBAD, VBO = 0xBAD, EBO = 0xBAD;
	static std::vector<float> vertices;
	static std::vector<unsigned int> indices;
	
	if (VAO == 0xBAD)
	{
		vertices = {
				-1.0f,  1.0f,
				1.0f,  1.0f,
				1.0f, -1.0f,
				-1.0f, -1.0f
		};
		
		indices = {
				0, 1, 2,
				2, 3, 0
		};
		
		glGenVertexArrays(1, &VAO);
		glBindVertexArray(VAO);
		
		glGenBuffers(1, &VBO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
		
		glGenBuffers(1, &EBO);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
		
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		
		glBindVertexArray(0);
	}
	
	shader->use();
	shader->setVec3("center", position);
	shader->setVec3("material.ambient", color);
	shader->setVec3("material.diffuse", Math::Vector3(0.9, 0.9, 0.9));
	shader->setVec3("material.specular", Math::Vector3(0, 0, 0));
	shader->setFloat("material.shininess", 0);
	shader->setFloat("radius", radius);
	
	glDisable(GL_CULL_FACE);
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, (int)indices.size(), GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}