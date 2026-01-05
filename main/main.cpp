#include "Arduino.h"
#include "FastAccelStepper.h"
#include "WiFi.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>

const char* ssid = CONFIG_ESP_WIFI_SSID;
const char* password = CONFIG_ESP_WIFI_PASSWORD;
int udp_socket;

#define LEG_A 0
#define LEG_B 1
#define LEG_C 2

#define EN_PIN D0 // Enable

int32_t pos[3] = {0, 0, 0};

//Global User Defined Constants
constexpr float base_length = 60;	  //distance from the center of the base to any of its corners
constexpr float platform_length = 90; //distance from the center of the platform to any of its corners
constexpr float leg1_length = 60;	  //length of link #1
constexpr float leg2_length = 80;	  //length of link #2

constexpr uint32_t steps_per_round = 200 * 64;
constexpr uint32_t steps_pi = steps_per_round / 2;
constexpr uint32_t steps_two_pi = steps_per_round;

constexpr float max_speed = 4 * steps_per_round;
constexpr float max_accel = max_speed / 0.16;

constexpr float steps_to_deg(int32_t steps) {
	return (steps / (float)steps_per_round) * 360.0;
}

constexpr int32_t deg_to_steps(float deg) {
	return (deg / 360.0) * steps_per_round;
}

constexpr int32_t rad_to_steps(float rad) {
	return rad * (steps_per_round / TWO_PI);
}

int32_t cleanup(float mag, float angle) {
	float v = (powf(mag, 2) + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * mag * leg1_length);
	float result = PI - (angle + acosf(min(1.0F, v)));
	result = constrain(result, -0.53F, 1.31F);
	return rad_to_steps(result);
}

constexpr float min_leg_length = leg2_length - leg1_length;
constexpr float max_leg_length = leg1_length + leg2_length;

constexpr float a0 = 1.5707288F * (steps_per_round / (float)TWO_PI);
constexpr float a1 = -0.2121144F * (steps_per_round / (float)TWO_PI);
constexpr float a2 = 0.0742610F * (steps_per_round / (float)TWO_PI);
constexpr float a3 = -0.0187293F * (steps_per_round / (float)TWO_PI);

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
		: [r] "=&f"(result),
		  [a] "=&f"(a),
		  [b] "=&f"(b),
		  [c] "=&f"(c),
		  [d] "=&f"(d),
		  [e] "=&f"(e),
		  [f] "=&f"(f)
		: [x] "f"(x)
	);
	return result;
}

static float rsqrtsf(float x) {
	float a, b, c, d;
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
		: [r] "=&f"(result),
		  [a] "=&f"(a),
		  [b] "=&f"(b),
		  [c] "=&f"(c),
		  [d] "=&f"(d)
		: [x] "f"(x)
	);
	return result;
}

static float fast_acos(float x) {
	float negate = float(x < 0);
	x = fabsf(x);
	float ret = a3;
	ret = ret * x + a2;
	ret = ret * x + a1;
	ret = ret * x + a0;
	ret = ret * sqrtsf(1.0F - x);
	ret = ret - 2 * negate * ret;
	return (negate * steps_pi) + ret;
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
	pos[0] = constrain((int32_t)result, rad_to_steps(-0.53F), rad_to_steps(1.31F));

	x = (sqrtf(3) * 0.5F) * (platform_length * (1 - (powf(nx, 2) + sqrtf(3) * nx * ny) / (nz + 1)) - base_length);
	y = x / sqrtf(3);
	z = hz - (platform_length / 2) * (sqrtf(3) * nx + ny);
	mag2 = powf(x, 2) + powf(y, 2) + powf(z, 2);
	if (mag2 >= max_leg_length * max_leg_length) { // leg wants to be longer than possible
		// reciprocal of magnitude, we're constraining it to the maximum leg length
		float rmag = 1 / max_leg_length;
		float angle = fast_acos(rmag * (sqrtf(3) * x + y) * -0.5F);
		result = steps_pi - angle;
	} else if (mag2 <= min_leg_length * min_leg_length) { // leg wants to be shorter than possible
		// reciprocal of magnitude, we're constraining it to the minimum leg length
		float rmag = 1 / min_leg_length;
		float angle = fast_acos(rmag * (sqrtf(3) * x + y) * -0.5F);

		// we know v = -1, and acos(-1) = pi, so pi - angle - pi = -angle
		result = -angle;
	} else {
		// reciprocal of magnitude, we're using the esp32-s3 rsqrt routine
		float rmag = rsqrtsf(mag2);
		float angle = fast_acos(rmag * (sqrtf(3) * x + y) * -0.5F);
		float v = rmag * (mag2 + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * leg1_length);
		result = steps_pi - angle - fast_acos(v);
	}
	pos[1] = constrain((int32_t)result, rad_to_steps(-0.53F), rad_to_steps(1.31F));

	x = (sqrtf(3) * 0.5F) * (base_length - platform_length * (1 - (powf(nx, 2) - sqrtf(3) * nx * ny) / (nz + 1)));
	y = -x / sqrtf(3);
	z = hz + (platform_length / 2) * (sqrtf(3) * nx - ny);
	mag2 = powf(x, 2) + powf(y, 2) + powf(z, 2);
	if (mag2 >= max_leg_length * max_leg_length) { // leg wants to be longer than possible
		// reciprocal of magnitude, we're constraining it to the maximum leg length
		float rmag = 1 / max_leg_length;
		float angle = fast_acos(rmag * (sqrtf(3) * x - y) * 0.5F);
		result = steps_pi - angle;
	} else if (mag2 <= min_leg_length * min_leg_length) { // leg wants to be shorter than possible
		// reciprocal of magnitude, we're constraining it to the minimum leg length
		float rmag = 1 / min_leg_length;
		float angle = fast_acos(rmag * (sqrtf(3) * x - y) * 0.5F);
		result = -angle;
	} else {
		float rmag = rsqrtsf(mag2); // reciprocal of magnitude, we use the esp32-s3 rsqrt routine
		float angle = fast_acos(rmag * (sqrtf(3) * x - y) * 0.5F);
		float v = rmag * (mag2 + powf(leg1_length, 2) - powf(leg2_length, 2)) / (2 * leg1_length);
		result = steps_pi - angle - fast_acos(v);
	}

	pos[2] = constrain((int32_t)result, rad_to_steps(-0.53F), rad_to_steps(1.31F));
}

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepperA = NULL;
FastAccelStepper* stepperB = NULL;
FastAccelStepper* stepperC = NULL;

constexpr int32_t start_position = deg_to_steps(-33.8355506);

__attribute__((always_inline)) inline FastAccelStepper* createStepper(uint8_t step, uint8_t dir, uint8_t enable) {
	auto* result = engine.stepperConnectToPin(step);

	if (result) {
		result->setDirectionPin(dir);
		result->setEnablePin(enable);

		result->setSpeedInUs(1000000 / max_speed); // the parameter is us/step !!!
		result->setAcceleration(max_accel);
		result->setCurrentPosition(start_position);
		result->enableOutputs();
	}

	return result;
}

void initWiFi() {
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	Serial.print("Connecting to WiFi ..");
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print('.');
		delay(1000);
	}
	Serial.println();
	Serial.println(WiFi.localIP());
}

void setup() {
	pinMode(D1, OUTPUT);
	digitalWrite(D1, LOW);
	pinMode(D2, OUTPUT);
	digitalWrite(D2, HIGH);
	digitalWrite(EN_PIN, HIGH);
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(115200);

	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	WiFi.mode(WIFI_STA);
	WiFi.disconnect();
	delay(100);

	initWiFi();

	udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	while (udp_socket == -1) {
		log_e("could not create socket: %d", errno);
		udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	}

	delay(1200);
	engine.init();

	stepperA = createStepper(D3, D4, D0);
	stepperB = createStepper(D5, D6, D0);
	stepperC = createStepper(D8, D7, D0);

	delay(1200);
}

struct SlewedValue {
	float oldValue;

	float calculate(float input, float elapsed, float slewLimit) {
		oldValue += constrain(input - oldValue, -slewLimit * elapsed, slewLimit * elapsed);
		return oldValue;
	}
};

struct Packet {
	int16_t lx, ly, lt, rt;
};

struct Controls {
	float lx, ly, lt, rt;
};

constexpr float heightmin = 45;
constexpr float heightmax = 136;
constexpr float heightmid = (heightmax + heightmin) / 2.0F;
constexpr float heightmap(float in) {
	return heightmid + (in * (heightmax - heightmid));
}

sockaddr_in recipient{
	.sin_family = AF_INET,
	.sin_port = htons(8888),
	.sin_addr = {.s_addr = (uint32_t)IPAddress(10, 0, 0, 250)},
};

const char ack = 'a';

Packet packet{};

int readpacket() {
	int len;

	if ((len = recv(udp_socket, &packet, sizeof(Packet), MSG_DONTWAIT)) == -1) {
		if (errno == EWOULDBLOCK) {
			return 0;
		}
		log_e("could not receive data: %d", errno);
		return 0;
	}

	return len;
}

void controller(float dt) {
	int sent = sendto(udp_socket, &ack, 1, 0, (struct sockaddr*)&recipient, sizeof(recipient));
	if (sent < 0) {
		log_e("could not send data: %d", errno);
	}

	int len = readpacket();
	if (len == sizeof(packet)) {
		Controls con{
			.lx = -packet.lx / 32768.0F,
			.ly = -packet.ly / 32768.0F,
			.lt = packet.lt / 32767.0F,
			.rt = packet.rt / 32767.0F,
		};

		float nx = con.lx / 3.0F;
		float ny = con.ly / 3.0F;
		float hz = heightmap(con.rt - con.lt);
		theta(hz, nx, ny);
		// pos[0] = deg_to_steps(constrain(180.f - theta(A, hz, nx, ny), -30.0f, 75.0f));
		// pos[1] = deg_to_steps(constrain(180.f - theta(B, hz, nx, ny), -30.0f, 75.0f));
		// pos[2] = deg_to_steps(constrain(180.f - theta(C, hz, nx, ny), -30.0f, 75.0f));
	}
}

size_t total_heap = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);

// uint64_t old = micros();
// uint64_t old1 = micros();
// uint16_t cnt = 0;
// constexpr uint64_t frequency = 240;
// constexpr uint64_t period = 1000000 / frequency;
// void loop() {
//   uint64_t now = micros();
//   float dt = (now - old) / 1000000.0f;

//   controller(dt);

//   stepperA->moveTo(pos[0]);
//   stepperB->moveTo(pos[1]);
//   stepperC->moveTo(pos[2]);

//   if (cnt % (uint16_t)(frequency / 1) == 0) {
//     Serial.println((frequency / 1) * 1000000 / (now - old1));
//     old1 = now;
//   }
//   cnt++;
//   old = now;

//   uint64_t end = micros();
//   if (end < (now + period - 40)) {
//     delayMicroseconds((now + period - 40) - end);
//   }
// }

uint64_t old = micros();
uint64_t old1 = micros();
uint16_t cnt = 0;
uint16_t cnt2 = 0;
uint32_t failcnt = 0;
uint64_t duration = 0;
constexpr uint64_t frequency = 240;
constexpr uint64_t period = 1000000 / frequency;
void loop() {
	uint64_t now = micros();
	float dt = (now - old) / 1000000.0F;

	int sent = sendto(udp_socket, &ack, 1, 0, (struct sockaddr*)&recipient, sizeof(recipient));
	if (sent < 0) {
		// log_e("could not send data: %d", errno);
		failcnt++;
	}

	int len = readpacket();
	if (len == sizeof(packet)) {
		Controls con{
			.lx = -packet.lx / 32768.0F,
			.ly = -packet.ly / 32768.0F,
			.lt = packet.lt / 32767.0F,
			.rt = packet.rt / 32767.0F,
		};

		float nx = con.lx / 3.0F;
		float ny = con.ly / 3.0F;
		float hz = heightmap(con.rt - con.lt);

		uint64_t start = micros();
		theta(hz, nx, ny);
		uint64_t end = micros();
		cnt2++;
		duration += end - start;
	}

	stepperA->moveTo(pos[0]);
	stepperB->moveTo(pos[1]);
	stepperC->moveTo(pos[2]);

	if (cnt % (uint16_t)(frequency / 1) == 0) {
		if (cnt2 != 0) {
			Serial.println(duration / cnt2);
		}
		size_t used_heap = total_heap - heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
		size_t used_psram = total_psram - heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
		Serial.printf("heap:  %7zu / %7zu (%zu)\n", used_heap, total_heap, (100 * used_heap) / total_heap);

		if (total_psram == 0) {
			Serial.printf("could not find psram!\n");
		} else {
			Serial.printf("psram: %7zu / %7zu (%zu)\n", used_psram, total_psram, (100 * used_psram) / total_psram);
		}

		Serial.printf("fail count: %ld\n", failcnt);

		old1 = now;
		duration = 0;
		cnt2 = 0;
	}
	cnt++;
	old = now;

	uint64_t end = micros();
	if (end < (now + period - 40)) {
		delayMicroseconds((now + period - 40) - end);
	}
}