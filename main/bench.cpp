#include <Arduino.h>
#include <cstdint>
#include <math.h>
#include "esp_random.h"

constexpr size_t N = 4096;

struct Input {
	float hz, nx, ny;
};

union {
	int32_t ints[N * 3];
	float floats[N * 3];
	Input inputs[N];
} data;

static float sqrtsf(float x) {
	float a, b, c, d, e, f;
	float result;
	asm volatile(
		"sqrt0.s %[a], %[x]\n"
		"const.s %[d], 0\n"
		"maddn.s %[d], %[a], %[a]\n"
		"nexp01.s %[b], %[x]\n"
		"const.s %[c], 3\n"
		"addexp.s %[b], %[c]\n"
		"maddn.s %[c], %[d], %[b]\n"
		"nexp01.s %[d], %[x]\n"
		"neg.s %[e], %[d]\n"
		"maddn.s %[a], %[c], %[a]\n"
		"const.s %[r], 0\n"
		"const.s %[c], 0\n"
		"const.s %[f], 0\n"
		"maddn.s %[r], %[e], %[a]\n"
		"maddn.s %[c], %[a], %[b]\n"
		"const.s %[e], 3\n"
		"maddn.s %[f], %[e], %[a]\n"
		"maddn.s %[d], %[r], %[r]\n"
		"maddn.s %[e], %[c], %[a]\n"
		"neg.s %[b], %[f]\n"
		"maddn.s %[r], %[d], %[b]\n"
		"maddn.s %[f], %[e], %[f]\n"
		"mksadj.s %[a], %[x]\n"
		"nexp01.s %[d], %[x]\n"
		"maddn.s %[d], %[r], %[r]\n"
		"neg.s %[b], %[f]\n"
		"addexpm.s %[r], %[a]\n"
		"addexp.s %[b], %[a]\n"
		"divn.s %[r], %[d], %[b]\n"
		: [r]"=&f"(result),
		  [a]"=&f"(a),
		  [b]"=&f"(b),
		  [c]"=&f"(c),
		  [d]"=&f"(d),
		  [e]"=&f"(e),
		  [f]"=&f"(f)
		: [x]"f"(x)
	);
	return result;
}

static float rsqrtsf(float x) {
	float a,b,c,d;
	float result;
	asm volatile(
		"rsqrt0.s	%[r], %[x]\n"
		"mul.s		%[a], %[x], %[r]\n"
		"const.s	%[b], 3\n"
		"mul.s		%[c], %[b], %[r]\n"
		"const.s	%[d], 1\n"
		"msub.s		%[d], %[a], %[r]\n"
		"madd.s		%[r], %[c], %[d]\n"
		"mul.s		%[a], %[x], %[r]\n"
		"mul.s		%[c], %[b], %[r]\n"
		"const.s	%[d], 1\n"
		"msub.s		%[d], %[a], %[r]\n"
		"maddn.s	%[r], %[c], %[d]\n"
		: [r]"=&f"(result),
		  [a]"=&f"(a),
		  [b]"=&f"(b),
		  [c]"=&f"(c),
		  [d]"=&f"(d)
		: [x]"f"(x)
	);
	return result;
}

static float sqrtsf2(float x) {
	float a,b,c,d;
	float result;
	asm volatile(
		"rsqrt0.s	%[r], %[x]\n"
		"mul.s		%[a], %[x], %[r]\n"
		"const.s	%[b], 3\n"
		"mul.s		%[c], %[b], %[r]\n"
		"const.s	%[d], 1\n"
		"msub.s		%[d], %[a], %[r]\n"
		"madd.s		%[r], %[c], %[d]\n"
		"mul.s		%[a], %[x], %[r]\n"
		"mul.s		%[c], %[b], %[r]\n"
		"const.s	%[d], 1\n"
		"msub.s		%[d], %[a], %[r]\n"
		"maddn.s	%[r], %[c], %[d]\n"
		: [r]"=&f"(result),
		  [a]"=&f"(a),
		  [b]"=&f"(b),
		  [c]"=&f"(c),
		  [d]"=&f"(d)
		: [x]"f"(x)
	);
	return result*x;
}

int32_t pos[3] = {0, 0, 0};

constexpr float base_length = 60;   //distance from the center of the base to any of its corners
constexpr float platform_length = 90; //distance from the center of the platform to any of its corners
constexpr float leg1_length = 60;   //length of link #1
constexpr float leg2_length = 80;   //length of link #2
constexpr float min_leg_length = leg2_length - leg1_length;
constexpr float max_leg_length = leg1_length + leg2_length;


constexpr uint32_t steps_per_round = 200 * 64;
constexpr uint32_t steps_pi = steps_per_round / 2;
constexpr uint32_t steps_two_pi = steps_per_round;

constexpr int32_t rad_to_steps(float rad) {
	return rad * (steps_per_round / (float)TWO_PI);
}

static void theta_baseline(float hz, float nx, float ny) {
	//create unit normal vector
	float nmag = sqrtf(powf(nx, 2) + powf(ny, 2) + 1); //magnitude of the normal vector
	nx /= nmag;
	ny /= nmag;
	float nz = 1 / nmag;

	float x, y, z; //generic variables for the components of legs A, B, and C
	float mag;    //generic magnitude of the leg vector
	float angle;   //generic angle for legs A, B, and C
	float v, result;
	//calculates angle A, B, or C

	y = base_length + (platform_length / 2) * (1 - (powf(nx, 2) + 3 * powf(nz, 2) + 3 * nz) / (nz + 1 - powf(nx, 2) + (powf(nx, 4) - 3 * powf(nx, 2) * powf(ny, 2)) / ((nz + 1) * (nz + 1 - powf(nx, 2)))));
	z = hz + platform_length * ny;
	mag = sqrtf(powf(y, 2) + powf(z, 2));
	mag = constrain(mag, min_leg_length, max_leg_length);
	angle = acosf(y / mag);
	v = (powf(y, 2) + powf(z, 2) + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * mag * leg1_length);
	result = PI - (angle + acosf(constrain(v, -1.0f, 1.0f)));
	result = constrain(result, -0.53f, 1.31f);
	pos[0] = rad_to_steps(result);

	x = (sqrtf(3) / 2) * (platform_length * (1 - (powf(nx, 2) + sqrtf(3) * nx * ny) / (nz + 1)) - base_length);
	y = x / sqrtf(3);
	z = hz - (platform_length / 2) * (sqrtf(3) * nx + ny);
	mag = sqrtf(powf(x, 2) + powf(y, 2) + powf(z, 2));
	mag = constrain(mag, min_leg_length, max_leg_length);
	angle = acosf((sqrtf(3) * x + y) / (-2 * mag));
	v = (powf(x, 2) + powf(y, 2) + powf(z, 2) + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * mag * leg1_length);
	result = PI - (angle + acosf(constrain(v, -1.0f, 1.0f)));
	result = constrain(result, -0.53f, 1.31f);
	pos[1] = rad_to_steps(result);

	x = (sqrtf(3) / 2) * (base_length - platform_length * (1 - (powf(nx, 2) - sqrtf(3) * nx * ny) / (nz + 1)));
	y = -x / sqrtf(3);
	z = hz + (platform_length / 2) * (sqrtf(3) * nx - ny);
	mag = sqrtf(powf(x, 2) + powf(y, 2) + powf(z, 2));
	mag = constrain(mag, min_leg_length, max_leg_length);
	angle = acosf((sqrtf(3) * x - y) / (2 * mag));
	v = (powf(x, 2) + powf(y, 2) + powf(z, 2) + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * mag * leg1_length);
	result = PI - (angle + acosf(constrain(v, -1.0f, 1.0f)));
	result = constrain(result, -0.53f, 1.31f);
	pos[2] = rad_to_steps(result);
}

constexpr float a0 = 1.5707288f * (steps_per_round / (float)TWO_PI);
constexpr float a1 = -0.2121144f * (steps_per_round / (float)TWO_PI);
constexpr float a2 = 0.0742610f * (steps_per_round / (float)TWO_PI);
constexpr float a3 = -0.0187293f * (steps_per_round / (float)TWO_PI);

static float fast_acos(float x) {
	float negate = float(x < 0);
	x = fabsf(x);
	float ret = a3;
	ret = ret * x + a2;
	ret = ret * x + a1;
	ret = ret * x + a0;
	ret = ret * sqrtsf(1.0f - x);
	ret = ret - 2 * negate * ret;
	return negate * steps_pi + ret;
}


static float fast_acos2(float x) {
	float negate = float(x < 0);
	x = fabsf(x);
	float ret = a3;
	ret = ret * x + a2;
	ret = ret * x + a1;
	ret = ret * x + a0;
	ret = ret * sqrtsf2(1.0f - x);
	ret = ret - 2 * negate * ret;
	return negate * steps_pi + ret;
}

static void theta(float hz, float nx, float ny) {
	//create unit normal vector
	float nz = rsqrtsf(powf(nx, 2) + powf(ny, 2) + 1); // reciprocal of the magnitude of the normal vector
	nx *= nz;
	ny *= nz;

	// we define these here, but the values aren't reused
	float x, y, z;
	float mag2; // square of the magnitude of the leg vector
	float result;

	y = base_length + (platform_length / 2) * (1 - (powf(nx, 2) + 3 * powf(nz, 2) + 3 * nz) / (nz + 1 - powf(nx, 2) + (powf(nx, 4) - 3 * powf(nx, 2) * powf(ny, 2)) / ((nz + 1) * (nz + 1 - powf(nx, 2)))));
	z = hz + platform_length * ny;
	// usually we'd calculate the actual magnitude, but since we constrain it to constants, using mag2 saves a sqrt when it's outside the allowed range
	mag2 = powf(y, 2) + powf(z, 2); 

	if (mag2 >= max_leg_length * max_leg_length) { // leg wants to be longer than possible
		// reciprocal of magnitude, we're constraining it to the maximum leg length
		float rmag = 1 / max_leg_length;
		float angle = fast_acos(y * rmag);

		// we know v = 1, and acos(1) = 0, so pi - angle - acos(v) = pi - angle
		result = steps_pi - angle;
	} else if (mag2 <= min_leg_length * min_leg_length) { // leg wants to be shorter than possible
		// reciprocal of magnitude, we're constraining it to the minimum leg length
		float rmag = 1 / min_leg_length;
		float angle = fast_acos(y * rmag);

		// we know v = -1, and acos(-1) = pi, so pi - angle - acos(v) = -angle
		result = -angle; 
	} else { // leg is within allowed range, we'll calculate it normally
		// reciprocal of magnitude, we use the esp32-s3 rsqrt routine
		float rmag = rsqrtsf(mag2);
		float angle = fast_acos(y * rmag);
		float v = rmag * (mag2 + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * leg1_length);

		result = steps_pi - angle - fast_acos(v);
	}
	pos[0] = constrain((int32_t)result, rad_to_steps(-0.53f), rad_to_steps(1.31f));

	x = (sqrtf(3) * 0.5f) * (platform_length * (1 - (powf(nx, 2) + sqrtf(3) * nx * ny) / (nz + 1)) - base_length);
	y = x / sqrtf(3);
	z = hz - (platform_length / 2) * (sqrtf(3) * nx + ny);
	mag2 = powf(x, 2) + powf(y, 2) + powf(z, 2);
	if (mag2 >= max_leg_length * max_leg_length) { // leg wants to be longer than possible
		// reciprocal of magnitude, we're constraining it to the maximum leg length
		float rmag = 1 / max_leg_length;
		float angle = fast_acos(rmag * (sqrtf(3) * x + y) * -0.5f);
		result = steps_pi - angle;
	} else if (mag2 <= min_leg_length * min_leg_length) { // leg wants to be shorter than possible
		// reciprocal of magnitude, we're constraining it to the minimum leg length
		float rmag = 1 / min_leg_length;
		float angle = fast_acos(rmag * (sqrtf(3) * x + y) * -0.5f);

		// we know v = -1, and acos(-1) = pi, so pi - angle - pi = -angle
		result = -angle;
	} else {
		// reciprocal of magnitude, we're using the esp32-s3 rsqrt routine
		float rmag = rsqrtsf(mag2);
		float angle = fast_acos(rmag * (sqrtf(3) * x + y) * -0.5f);
		float v = rmag * (mag2 + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * leg1_length);
		result = steps_pi - angle - fast_acos(v);
	}
	pos[1] = constrain((int32_t)result, rad_to_steps(-0.53f), rad_to_steps(1.31f));

	x = (sqrtf(3) * 0.5f) * (base_length - platform_length * (1 - (powf(nx, 2) - sqrtf(3) * nx * ny) / (nz + 1)));
	y = -x / sqrtf(3);
	z = hz + (platform_length / 2) * (sqrtf(3) * nx - ny);
	mag2 = powf(x, 2) + powf(y, 2) + powf(z, 2);
	if (mag2 >= max_leg_length * max_leg_length) { // leg wants to be longer than possible
		// reciprocal of magnitude, we're constraining it to the maximum leg length
		float rmag = 1 / max_leg_length;
		float angle = fast_acos(rmag * (sqrtf(3) * x - y) * 0.5f);
		result = steps_pi - angle;
	} else if (mag2 <= min_leg_length * min_leg_length) { // leg wants to be shorter than possible
		// reciprocal of magnitude, we're constraining it to the minimum leg length
		float rmag = 1 / min_leg_length;
		float angle = fast_acos(rmag * (sqrtf(3) * x - y) * 0.5f);
		result = -angle;
	} else {
		float rmag = rsqrtsf(mag2); // reciprocal of magnitude, we use the esp32-s3 rsqrt routine
		float angle = fast_acos(rmag * (sqrtf(3) * x - y) * 0.5f);
		float v = rmag * (mag2 + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * leg1_length);
		result = steps_pi - angle - fast_acos(v);
	}

	pos[2] = constrain((int32_t)result, rad_to_steps(-0.53f), rad_to_steps(1.31f));
}


static void theta2(float hz, float nx, float ny) {
	//create unit normal vector
	float nz = rsqrtsf(powf(nx, 2) + powf(ny, 2) + 1); // reciprocal of the magnitude of the normal vector
	nx *= nz;
	ny *= nz;

	// we define these here, but the values aren't reused
	float x, y, z;
	float mag2; // square of the magnitude of the leg vector
	float result;

	y = base_length + (platform_length / 2) * (1 - (powf(nx, 2) + 3 * powf(nz, 2) + 3 * nz) / (nz + 1 - powf(nx, 2) + (powf(nx, 4) - 3 * powf(nx, 2) * powf(ny, 2)) / ((nz + 1) * (nz + 1 - powf(nx, 2)))));
	z = hz + platform_length * ny;
	// usually we'd calculate the actual magnitude, but since we constrain it to constants, using mag2 saves a sqrt when it's outside the allowed range
	mag2 = powf(y, 2) + powf(z, 2); 

	if (mag2 >= max_leg_length * max_leg_length) { // leg wants to be longer than possible
		// reciprocal of magnitude, we're constraining it to the maximum leg length
		float rmag = 1 / max_leg_length;
		float angle = fast_acos2(y * rmag);

		// we know v = 1, and acos(1) = 0, so pi - angle - acos(v) = pi - angle
		result = steps_pi - angle;
	} else if (mag2 <= min_leg_length * min_leg_length) { // leg wants to be shorter than possible
		// reciprocal of magnitude, we're constraining it to the minimum leg length
		float rmag = 1 / min_leg_length;
		float angle = fast_acos2(y * rmag);

		// we know v = -1, and acos(-1) = pi, so pi - angle - acos(v) = -angle
		result = -angle; 
	} else { // leg is within allowed range, we'll calculate it normally
		// reciprocal of magnitude, we use the esp32-s3 rsqrt routine
		float rmag = rsqrtsf(mag2);
		float angle = fast_acos2(y * rmag);
		float v = rmag * (mag2 + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * leg1_length);

		result = steps_pi - angle - fast_acos2(v);
	}
	pos[0] = constrain((int32_t)result, rad_to_steps(-0.53f), rad_to_steps(1.31f));

	x = (sqrtf(3) * 0.5f) * (platform_length * (1 - (powf(nx, 2) + sqrtf(3) * nx * ny) / (nz + 1)) - base_length);
	y = x / sqrtf(3);
	z = hz - (platform_length / 2) * (sqrtf(3) * nx + ny);
	mag2 = powf(x, 2) + powf(y, 2) + powf(z, 2);
	if (mag2 >= max_leg_length * max_leg_length) { // leg wants to be longer than possible
		// reciprocal of magnitude, we're constraining it to the maximum leg length
		float rmag = 1 / max_leg_length;
		float angle = fast_acos2(rmag * (sqrtf(3) * x + y) * -0.5f);
		result = steps_pi - angle;
	} else if (mag2 <= min_leg_length * min_leg_length) { // leg wants to be shorter than possible
		// reciprocal of magnitude, we're constraining it to the minimum leg length
		float rmag = 1 / min_leg_length;
		float angle = fast_acos2(rmag * (sqrtf(3) * x + y) * -0.5f);

		// we know v = -1, and acos(-1) = pi, so pi - angle - pi = -angle
		result = -angle;
	} else {
		// reciprocal of magnitude, we're using the esp32-s3 rsqrt routine
		float rmag = rsqrtsf(mag2);
		float angle = fast_acos2(rmag * (sqrtf(3) * x + y) * -0.5f);
		float v = rmag * (mag2 + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * leg1_length);
		result = steps_pi - angle - fast_acos2(v);
	}
	pos[1] = constrain((int32_t)result, rad_to_steps(-0.53f), rad_to_steps(1.31f));

	x = (sqrtf(3) * 0.5f) * (base_length - platform_length * (1 - (powf(nx, 2) - sqrtf(3) * nx * ny) / (nz + 1)));
	y = -x / sqrtf(3);
	z = hz + (platform_length / 2) * (sqrtf(3) * nx - ny);
	mag2 = powf(x, 2) + powf(y, 2) + powf(z, 2);
	if (mag2 >= max_leg_length * max_leg_length) { // leg wants to be longer than possible
		// reciprocal of magnitude, we're constraining it to the maximum leg length
		float rmag = 1 / max_leg_length;
		float angle = fast_acos2(rmag * (sqrtf(3) * x - y) * 0.5f);
		result = steps_pi - angle;
	} else if (mag2 <= min_leg_length * min_leg_length) { // leg wants to be shorter than possible
		// reciprocal of magnitude, we're constraining it to the minimum leg length
		float rmag = 1 / min_leg_length;
		float angle = fast_acos2(rmag * (sqrtf(3) * x - y) * 0.5f);
		result = -angle;
	} else {
		float rmag = rsqrtsf(mag2); // reciprocal of magnitude, we use the esp32-s3 rsqrt routine
		float angle = fast_acos2(rmag * (sqrtf(3) * x - y) * 0.5f);
		float v = rmag * (mag2 + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * leg1_length);
		result = steps_pi - angle - fast_acos2(v);
	}

	pos[2] = constrain((int32_t)result, rad_to_steps(-0.53f), rad_to_steps(1.31f));
}

constexpr float heightmin = 45 - 5;
constexpr float heightmax = 136 + 5;
constexpr float heightmid = (heightmax + heightmin) / 2.0f;
constexpr float heightmap(float in) {
	return heightmid + in * (heightmax - heightmid);
}

void realistic_data() {
	for (int i = 0; i < N; i++) {
		float t = i * (2 * (float)PI / (float)N);

		data.inputs[i] = {
			.hz = heightmap(sinf(2 * t)),
			.nx = cosf(t),
			.ny = sinf(t),
		};
	}
}

void benchmark() {
	uint64_t duration1 = 0;
	for (int i = 0; i < N; i++) {
		Input input = data.inputs[i];
		uint64_t start = micros();
		theta2(input.hz, input.nx, input.ny);
		duration1 += micros() - start;
		int32_t result0 = pos[0];
		int32_t result1 = pos[1];
		int32_t result2 = pos[2];
		asm volatile(""
					 : "+m,r"(result0), "+m,r"(result1), "+m,r"(result2)
					 :
					 : "memory");
	}
	uint64_t duration2 = 0;
	for (int i = 0; i < N; i++) {
		Input input = data.inputs[i];
		uint64_t start = micros();
		theta(input.hz, input.nx, input.ny);
		duration2 += micros() - start;
		int32_t result0 = pos[0];
		int32_t result1 = pos[1];
		int32_t result2 = pos[2];
		asm volatile(""
					 : "+m,r"(result0), "+m,r"(result1), "+m,r"(result2)
					 :
					 : "memory");
	}
	Serial.printf("theta1 took %.2f us", duration1 / (float)N);
	Serial.printf(" theta2 took %.2f us\n", duration2 / (float)N);
}

void measure_error() {
	int32_t max_error[3] = {0, 0, 0};
	int32_t total_error[3] = {0, 0, 0};
	for (int i = 0; i < N; i++) {
		Input const input = data.inputs[i];
		
		theta_baseline(input.hz, input.nx, input.ny);
		int32_t baseline[3] = {pos[0], pos[1], pos[2]};
		theta2(input.hz, input.nx, input.ny);
		int32_t result[3] = {pos[0], pos[1], pos[2]};

		for (int j = 0; j < 3; j++) {
			int32_t error = abs(baseline[j] - result[j]);
			total_error[j] += error;
			if (max_error[j] < error) {
				max_error[j] = error;
			}
		}
	}
	
	Serial.println("===============================================");
	Serial.println();

	int32_t overall_error = 0;
	for (int i = 0; i < 3; i++) {
		overall_error += total_error[i];

		Serial.printf("stepper[%d] avg error: %.3f, max error: %d\n", i, (float)total_error[i] / (float)N, max_error[i]);
	}

	Serial.printf("overall    avg error: %.3f\n", (float)overall_error / (float)(N * 3));

	Serial.flush();
}

void setup() {
	Serial.begin(115200);

	// esp_fill_random(data.ints, sizeof(data.ints));

	// for (int i = 0; i < N * 3; i++) {
	// 	float result = 2.0f * ((float)data.ints[i] / (float)INT32_MAX);
	// 	if (i % 3 == 0) result = heightmap(result);
	// 	data.floats[i] = result * 2;
	// }

	realistic_data();
	delay(1000);
	benchmark();
}

void loop() {
	benchmark();
	delay(100);
}
