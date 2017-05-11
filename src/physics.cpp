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

#pragma region variables

bool firstTime = true;
bool collision = false;

// Inertia variables
float mass = 1;

vec3 gravity(0.0, -9.8, 0.0);
vec3 appliedPoint(0.3, 0.0, 0.3);
float timeCount = 0.0;

vec3 position, velocity, angularMom, linealMom, torque, force, angularVel;
quat orientation;
mat3 I;
mat3 Ibody;

vec3 Pa, J;
float Vrel;
float j = 0.0;
float E = 0.3;
vec3 prevPos[8];
vec3 cubeVertsPos[8];

void Restart();

#pragma endregion


namespace Cube {
	extern void setupCube();
	extern void cleanupCube();
	extern void updateCube(mat4& transform);
	extern void drawCube();
	extern glm::vec3 cubeVerts[];
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

	bool firstTime = true;

	vec3 linearMom = vec3(0.f), angularMom = vec3(0.f), velocity = vec3(0.f), comPos = vec3(0.f), angularVel = vec3(0.f), torque = vec3(0.f), force;
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
	mat3 Inertia(quat rotation) {
		mat3 inertiaMat{ 
			(1.0 / 12.0) * mass * (height * height + depth * depth), 0.0, 0.0,
			0.0, (1.0 / 12.0) * mass * (width * width + depth * depth), 0.0,
			0.0, 0.0, (1.0 / 12.0) * mass * (width * width + height * height) };

		return mat3_cast(rotation) * inverse(inertiaMat) * transpose(mat3_cast(rotation));
	}

	//Angular Velocity
	vec3 AngularVelocity(mat3 inertia, vec3 angularM, float dt) {
		return inertia * angularM * dt;
	}

	//Rotation
	quat Orientation(quat orientation, vec3 angularV) {
		return normalize((orientation) + 0.5f * quat(0, angularV) * orientation);
	}

	//Cube fragments position
	mat4 Position(mat4 orientation, vec3 position, vec3 comPosition) {
		mat4 positionM = translate(positionM, position);
		mat4 comPositionM = translate(comPositionM, comPosition);
		return orientation * positionM + comPositionM;
	}

	mat4 TransformationMatrix(vec3 initialForce, vec3 collisionPnt, float dt, float mass) {
		if (firstTime) { 
			force = initialForce; 
			firstTime = false;
			torque = vec3(0.0);
			torque = Torque(comPos, force, collisionPnt);
			angularMom = AngularMomentum(angularMom, torque, dt);
		}
		else {
			force = vec3(0.0, -9.8, 0.0);
		}
		//torque = Torque(comPos, force, collisionPnt);
		linearMom = LinearMomentum(linearMom, force, dt);
		velocity = Velocity(linearMom, mass);
		comPos = CoMPosition(comPos, velocity, dt);
		inertia = Inertia(rotationVal);
		angularVel = AngularVelocity(inertia, angularMom, dt);
		rotationVal = Orientation(mat4_cast(rotationVal), angularVel);
		
		translationMat1 = translate(translationMat1, comPos);

		std::cout << comPos.y << std::endl;
		

		mat4 finalMatrix = transpose(mat4(mat3_cast(rotationVal))) * translationMat1;
		return finalMatrix;
	}

	void SetCoM(vec3 comPosition) {
		comPos = comPosition;
	}
};

void PhysicsInit() {
	//TODO
	appliedPoint = vec3(0.3, 0.0, 0.3);
	velocity = torque = linealMom = angularMom = vec3(0.0);
	force = vec3(5.0, 10.f, 0.0);
	position = vec3(0.0, 5.0, 0.0);
	orientation = quat();
	firstTime = true;
	Ibody = mat3((1.0 / 12.0) * mass * 2);
	
}
void PhysicsUpdate(float dt) {
	//Cube::updateCube(EOM.TransformationMatrix(vec3(0.0f, 100.0, 0.0f), vec3(0.3, 0.0, 0.3), dt, 1));
	if (ImGui::Button("Restart")) {
		Restart();
	}

	if (firstTime) {
		torque = cross(appliedPoint, force);
		linealMom = linealMom + force + gravity * dt;
		angularMom = angularMom + torque * dt;
		firstTime = false;
	}

	else if(!collision) {
		linealMom = linealMom + gravity * dt;
	}

	velocity = linealMom / mass;
	position = position + velocity * dt;
	
	mat3 orientMat = mat3_cast(orientation);
	I = orientMat * inverse(Ibody) * transpose(orientMat);
	angularVel = I * angularMom * dt;

	quat quatW(0, angularVel);
	orientation = normalize(orientation + quatW * orientation);

	mat4 rotation = transpose(mat3_cast(orientation));

	mat4 positionMat(
		1.0, 0.0, 0.0, position.x,
		0.0, 1.0, 0.0, position.y,
		0.0, 0.0, 1.0, position.z,
		0.0, 0.0, 0.0, 1.0);
	
	mat4 updateMatrix = transpose(positionMat) * rotation;
	
	collision = false;

	for (int i = 0; i < 8; ++i) {

		cubeVertsPos[i] = (mat3)rotation * Cube::cubeVerts[i] + position;
		/*if (cubeVertsPos[i].y < 0.0) {
			force.y = -velocity.y;
			appliedPoint = cubeVertsPos[i];
			torque = cross(appliedPoint - position, force);
			linealMom = torque;
			angularMom = inverse(I) * torque;
			collision = true;
		}
		if (cubeVertsPos[i].x > 5.0) {
			force.x = -velocity.x;
			appliedPoint = cubeVertsPos[i];
			torque = cross(appliedPoint - position, force);
			linealMom = torque;
			angularMom = inverse(I) * torque;
			collision = true;
		}
		if (cubeVertsPos[i].y > 10) {
			force.y = -velocity.y;
			appliedPoint = cubeVertsPos[i];
			torque = cross(appliedPoint - position, force);
			linealMom = torque;
			angularMom = inverse(I) * torque;
			collision = true;
		}
		if (cubeVertsPos[i].x < -5.0) {
			force.x = -velocity.x;
			appliedPoint = cubeVertsPos[i];
			torque = cross(appliedPoint - position, force);
			linealMom = torque;
			angularMom = inverse(I) * torque;
			collision = true;
		}*/

		if (cubeVertsPos[i].y < 0.0) {
			Pa = velocity + cross(angularVel, (cubeVertsPos[i] - position));
			Vrel = dot(vec3(0.0, 1.0, 0.0), Pa);
			j = (-(1 + E)*Vrel) / (1 / mass + dot(vec3(0.0, 1.0, 0.0), (inverse(I) * cross(cross(cubeVertsPos[i], vec3(0.0, 1.0, 0.0)), cubeVertsPos[i]))));
			J = j * vec3(0.0, 1.0, 0.0);
			torque = cross(cubeVertsPos[i], J);
			linealMom = -linealMom + J;
			angularMom = -angularMom + torque;
		}
		if (cubeVertsPos[i].x > 5.0) {
			Pa = velocity + cross(angularVel, (cubeVertsPos[i] - position));
			Vrel = dot(vec3(-1.0, 0.0, 0.0), Pa);
			j = (-(1 + E)*Vrel) / (1 / mass + dot(vec3(-1.0, 0.0, 0.0), (inverse(I) * cross(cross(cubeVertsPos[i], vec3(-1.0, 0.0, 0.0)), cubeVertsPos[i]))));
			J = j * vec3(-1.0, 0.0, 0.0);
			torque = cross(cubeVertsPos[i], J);
			linealMom = -linealMom + J;
			angularMom = -angularMom + torque;
		}
		if (cubeVertsPos[i].y > 10) {
			Pa = velocity + cross(angularVel, (cubeVertsPos[i] - position));
			Vrel = dot(vec3(0.0, -1.0, 0.0), Pa);
			j = (-(1 + E)*Vrel) / (1 / mass + dot(vec3(0.0, -1.0, 0.0), (inverse(I) * cross(cross(cubeVertsPos[i], vec3(0.0, -1.0, 0.0)), cubeVertsPos[i]))));
			J = j * vec3(0.0, -1.0, 0.0);
			torque = cross(cubeVertsPos[i], J);
			linealMom = -linealMom + J;
			angularMom = -angularMom + torque;
		}
		if (cubeVertsPos[i].x < -5.0) {
			Pa = velocity + cross(angularVel, (cubeVertsPos[i] - position));
			Vrel = dot(vec3(1.0, 0.0, 0.0), Pa);
			j = (-(1 + E)*Vrel) / (1 / mass + dot(vec3(1.0, 0.0, 0.0), (inverse(I) * cross(cross(cubeVertsPos[i], vec3(1.0, 0.0, 0.0)), cubeVertsPos[i]))));
			J = j * vec3(1.0, 0.0, 0.0);
			torque = cross(cubeVertsPos[i], J);
			linealMom = -linealMom + J;
			angularMom = -angularMom + torque;
		}

		prevPos[i] = cubeVertsPos[i];
	}

	std::cout << velocity.x << std::endl;
	Cube::updateCube(updateMatrix);
	timeCount += dt;
	//std::cout << timeCount << std::endl;

}
void PhysicsCleanup() {
	//TODO
}

void Restart() {
	appliedPoint = vec3(0.3, 0.0, 0.3);
	velocity = torque = linealMom = angularMom = vec3(0.0);
	force = vec3(5.0, 10.f, 0.0);
	position = vec3(0.0, 5.0, 0.0);
	orientation = quat();
	firstTime = true;
	Ibody = mat3((1.0 / 12.0) * mass * 2);
}