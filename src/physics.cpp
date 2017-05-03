#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\quaternion.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <iostream>


#define EOM EquationsOfMotion::Instance()

using namespace glm;

bool show_test_window = false;

mat4 cubeTransform;

namespace Cube {
	extern void setupCube();
	extern void cleanupCube();
	extern void updateCube(mat4& transform);
	extern void drawCube();
}

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		//TODO
	}

	// ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

class EquationsOfMotion {
private:
	
	// Inertia variables
	float mass = 1;
	float height = 1;
	float width = 1;
	float depth = 1;

	mat4 inertiaMat = scale(inertiaMat, vec3((1 / 12) * mass * (height * height + depth * depth), 
		(1 / 12) * mass * (width * width + depth * depth),
		(1 / 12) * mass * (width * width + depth * depth)));


	vec3 linearMom = vec3(0.f), angularMom = vec3(0.f), velocity = vec3(0.f), comPos = vec3(0.f), angularVel = vec3(0.f), torque = vec3(0.f);
	mat4 inertia, orientation, position, model, translationMat1, translationMat2;
	quat rotationVal;
public:
	static EquationsOfMotion& Instance() {
		static EquationsOfMotion equisde;
		return equisde;
	}

	//Torque
	vec3 Torque(vec3 comPosition, vec3 force, vec3 collisionPoint) {
		return cross((collisionPoint - comPosition), force);
	}

	//Linear Momentum
	vec3 LinearMomentum(vec3 linearM, vec3 force, float dt) {
		return linearM + dt * force;
	}

	//Angular Momentum
	vec3 AngularMomentum(vec3 angularM, vec3 torque, float dt) {
		return angularM + dt * torque;
	}

	//Velocity
	vec3 Velocity(vec3 linearM, float mass) {
		return linearM / mass;
	}

	//CoM Position
	vec3 CoMPosition(vec3 comPosition, vec3 velocity, float dt) {
		return comPosition + dt * velocity;
	}

	//Inertia
	mat4 Inertia(quat rotation) {
		return mat4_cast(rotation) * inverse(inertiaMat) * transpose(mat4_cast(rotation));
	}

	//Angular Velocity
	vec3 AngularVelocity(mat4 inertia, vec3 angularM, float dt) {
		return inertia * vec4(angularM, 1.f);
	}

	//Rotation
	mat4 Orientation(mat4 orientation, vec3 angularV, float dt) {
		return (orientation)+dt * mat4_cast(quat(vec4(angularV, 1.f)) * quat(orientation));
	}

	//Cube fragments position
	mat4 Position(mat4 orientation, vec3 position, vec3 comPosition) {
		mat4 positionM = translate(positionM, position);
		mat4 comPositionM = translate(comPositionM, comPosition);
		return orientation * positionM + comPositionM;
	}

	mat4 TransformationMatrix(vec3 force, vec3 collisionPnt, float dt, float mass) {
		torque = vec3(0.0);
		torque = Torque(comPos, force, collisionPnt);
		linearMom = LinearMomentum(linearMom, force, dt);
		angularMom = AngularMomentum(angularMom, torque, dt);
		velocity = Velocity(linearMom, mass);
		comPos = CoMPosition(comPos, velocity, dt);
		inertia = Inertia(rotationVal);
		angularVel = AngularVelocity(inertia, angularMom, dt);
		rotationVal = Orientation(mat4_cast(rotationVal), angularVel, dt);
		
		translationMat1 = translate(translationMat1, comPos);

		mat4 finalMatrix = mat4(rotationVal) * translationMat1;
		return finalMatrix;
	}

	void SetCoM(vec3 comPosition) {
		comPos = comPosition;
	}
};

void PhysicsInit() {
	//TODO	

}
void PhysicsUpdate(float dt) {
	Cube::updateCube(EOM.TransformationMatrix(vec3(0.1f, 0.1f, 0.1f), vec3(0.0f), dt, 1));
	
}
void PhysicsCleanup() {
	//TODO
}