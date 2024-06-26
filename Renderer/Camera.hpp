#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <Vector3.hpp>
#include <Matrix4.hpp>

class Camera {
	private:
	    Math::Matrix4 yRotation;
	    Math::Matrix4 xRotation;
	    Math::Matrix4 originTranslation;
	    
	    Math::Matrix4 targetTranslation;
	
	public:
	    Math::Scalar phi;
	    Math::Scalar theta;
	    Math::Scalar ro;
	    
	    Math::Scalar orthographicScale;
	    
	    Camera();
	    
	    void setTarget(const Math::Vector3& target);
	    
	    void rotateAroundY(Math::Scalar delta);
	    void rotateAroundX(Math::Scalar delta);
	    void resetRotation();
	    
	    void translate(Math::Scalar delta);
	    void resetTranslation();
	    
	    void scale(Math::Scalar scale);
	    void resetScale();
	    
	    Math::Matrix4 getViewMatrix();
	    Math::Matrix4 getPerspectiveMatrix(Math::Scalar fov, Math::Scalar aspect, Math::Scalar near, Math::Scalar far) const;
	    Math::Matrix4 getOrthographicMatrix(Math::Scalar width, Math::Scalar heigh, Math::Scalar near, Math::Scalar far) const;
};

#endif
