#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\quaternion.hpp>

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
	static EquationsOfMotion s_instance;

	static EquationsOfMotion& Instance() {
		return s_instance;
	}

public:

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

	//Position
	vec3 Position(vec3 position, vec3 velocity, float dt) {
		return position + dt * velocity;
	}

	//Inertia
	mat4 Inertia(mat4 Ibody, quat rotation) {
		return mat4_cast(rotation) * inverse(Ibody) * transpose(mat4_cast(rotation));
	}

	//Angular Velocity
	vec3 AngularVelocity(mat4 inertia, vec3 angularM, float dt) {
		return inertia * vec4(angularM, 1.f);
	}

	//Rotation
	quat Rotation(quat rotation, vec3 angularV, float dt){
		return mat4_cast(rotation) + dt * (vec4(angularV, 1.f) * mat4_cast(rotation));
	}

};

void PhysicsInit() {
	//TODO
}
void PhysicsUpdate(float dt) {
	Cube::updateCube(cubeTransform);
	//TODO
}
void PhysicsCleanup() {
	//TODO
}