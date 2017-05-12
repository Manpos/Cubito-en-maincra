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

float mass = 1;

vec3 gravity(0.0, -9.8, 0.0);
vec3 appliedPoint(0.3, 0.0, 0.3);
float timeCount = 0.0;
float bounceTolerance = 1;

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

void PhysicsInit() {
	
	float HI = 5, LO = -5;
	float r1 = LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HI - LO)));
	float r2 = 5 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (7 - 5)));
	float r3 = LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HI - LO)));


	float ascendent = 5 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (10 - 5)));

	float HAP = 1, LAP = -1;
	float appPointX = LAP + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HAP - LAP)));
	float appPointZ = LAP + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HAP - LAP)));

	position = { r1, r2, r3 };

	force = vec3(0.0, ascendent, 0.0);
	velocity = torque = linealMom = angularMom = vec3(0.0);
	appliedPoint = vec3(position.x + appPointX, 0.0, position.z + appPointZ);
	orientation = quat();
	firstTime = true;
	Ibody = mat3((1.0 / 12.0) * mass * 2);

	timeCount = 0;
}

void PhysicsUpdate(float dt) {

	if (ImGui::Button("Restart") || timeCount >= 5) {
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
	
	for (int i = 0; i < 8; ++i) {

		cubeVertsPos[i] = (mat3)rotation * Cube::cubeVerts[i] + position;

// Here the code of the collisions we tried to implement
#pragma region Collisions

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

		/*if (cubeVertsPos[i].y < 0.0) {
		Pa = velocity + cross(angularVel, (cubeVertsPos[i] - position));
		Vrel = dot(vec3(0.0, 1.0, 0.0), Pa);
		j = -((E) * Vrel);
		J = j * vec3(0.0, 1.0, 0.0);
		torque = cross(cubeVertsPos[i], J);
		linealMom = linealMom + J;
		angularMom = angularMom + torque;*/
		/*j = -((1 + E)*Vrel) / (1 / mass + dot(vec3(0.0, 1.0, 0.0), (inverse(I) * cross(cross(cubeVertsPos[i], vec3(0.0, 1.0, 0.0)), cubeVertsPos[i]))));
		J = j * vec3(0.0, 1.0, 0.0);
		torque = cross(cubeVertsPos[i], J);
		linealMom = -linealMom + J;
		angularMom = -angularMom + torque;*/
	}

	/*if (cubeVertsPos[i].x > 5.0) {
	Pa = velocity + cross(angularVel, (cubeVertsPos[i] - position));
	Vrel = dot(vec3(-1.0, 0.0, 0.0), Pa);
	j = (-(1 + E)*Vrel) / (1 / mass + dot(vec3(-1.0, 0.0, 0.0), (inverse(I) * cross(cross(cubeVertsPos[i], vec3(-1.0, 0.0, 0.0)), cubeVertsPos[i]))));
	J = j * vec3(-1.0, 0.0, 0.0);
	torque = cross(cubeVertsPos[i], J);
	linealMom = linealMom + J;
	angularMom = angularMom + torque;
	collision = true;
	}
	if (cubeVertsPos[i].y > 10) {
	Pa = velocity + cross(angularVel, (cubeVertsPos[i] - position));
	Vrel = dot(vec3(0.0, -1.0, 0.0), Pa);
	j = (-(1 + E)*Vrel) / (1 / mass + dot(vec3(0.0, -1.0, 0.0), (inverse(I) * cross(cross(cubeVertsPos[i], vec3(0.0, -1.0, 0.0)), cubeVertsPos[i]))));
	J = j * vec3(0.0, -1.0, 0.0);
	torque = cross(cubeVertsPos[i], J);
	linealMom = linealMom + J;
	angularMom = angularMom + torque;
	collision = true;
	}
	if (cubeVertsPos[i].x < -5.0) {
	Pa = velocity + cross(angularVel, (cubeVertsPos[i] - position));
	Vrel = dot(vec3(1.0, 0.0, 0.0), Pa);
	j = (-(1 + E)*Vrel) / (1 / mass + dot(vec3(1.0, 0.0, 0.0), (inverse(I) * cross(cross(cubeVertsPos[i], vec3(1.0, 0.0, 0.0)), cubeVertsPos[i]))));
	J = j * vec3(1.0, 0.0, 0.0);
	torque = cross(cubeVertsPos[i], J);
	linealMom = linealMom + J;
	angularMom = angularMom + torque;
	collision = true;
	}*/

	//	prevPos[i] = cubeVertsPos[i];
	//}
#pragma endregion
	
	Cube::updateCube(updateMatrix);
	timeCount += dt;
}
void PhysicsCleanup() {
	//TODO
}

void Restart() {

	float HI = 5, LO = -5;
	float r1 = LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HI - LO)));
	float r2 = 5 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (7 - 5)));
	float r3 = LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HI - LO)));


	float ascendent = 5 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (10 - 5)));
	
	float HAP = 1, LAP = -1;
	float appPointX = LAP + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HAP - LAP)));
	float appPointZ = LAP + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HAP - LAP)));

	position = { r1, r2, r3 };

	force = vec3(0.0, ascendent, 0.0);
	velocity = torque = linealMom = angularMom = vec3(0.0);
	appliedPoint = vec3(position.x + appPointX, 0.0, position.z + appPointZ);
	orientation = quat();
	firstTime = true;
	Ibody = mat3((1.0 / 12.0) * mass * 2);

	timeCount = 0;
}