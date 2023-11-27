#include "vector.h"

vector2f vector2fAdd(const vector2f vec1, const vector2f vec2) {
	vector2f vec = {
		.x = vec1.x + vec2.x,
		.y = vec1.y + vec2.y,
	};
	return vec;
}
vector2f vector2fSub(const vector2f vec1, const vector2f vec2) {
	vector2f vec = {
		.x = vec1.x - vec2.x,
		.y = vec1.y - vec2.y,
	};
	return vec;
}
vector2f vector2fMulty(const vector2f vec1, const float num) {
	vector2f vec = {
		.x = vec1.x * num,
		.y = vec1.y * num,
	};
	return vec;
}
vector2f vector2fDivid(const vector2f vec1, const float num) {
	if (num == 0)
		return vec1;
	vector2f vec = {
		.x = vec1.x / num,
		.y = vec1.y / num,
	};
	return vec;
}
float vector2fDot(const vector2f vec1, const vector2f vec2) { return vec1.x * vec2.x + vec1.y * vec2.y; }

vector3f vector3fAdd(const vector3f vec1, const vector3f vec2) {
	vector3f vec = {
		.x = vec1.x + vec2.x,
		.y = vec1.y + vec2.y,
		.z = vec1.z + vec2.z,
	};
	return vec;
}
vector3f vector3fSub(const vector3f vec1, const vector3f vec2) {
	vector3f vec = {
		.x = vec1.x - vec2.x,
		.y = vec1.y - vec2.y,
		.z = vec1.z - vec2.z,
	};
	return vec;
}
vector3f vector3fMulty(const vector3f vec1, const float num) {
	vector3f vec = {
		.x = vec1.x * num,
		.y = vec1.y * num,
		.z = vec1.z * num,
	};
	return vec;
}
vector3f vector3fDivid(const vector3f vec1, const float num) {
	if (num == 0)
		return vec1;
	vector3f vec = {
		.x = vec1.x / num,
		.y = vec1.y / num,
		.z = vec1.z / num,
	};
	return vec;
}
float vector3fDot(const vector3f vec1, const vector3f vec2) { return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z; }
