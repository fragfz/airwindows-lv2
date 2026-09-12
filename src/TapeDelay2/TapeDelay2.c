#include <lv2/core/lv2.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define TAPEDELAY2_URI "https://hannesbraun.net/ns/lv2/airwindows/tapedelay2"

typedef enum {
	INPUT_L = 0,
	INPUT_R = 1,
	OUTPUT_L = 2,
	OUTPUT_R = 3,
	TIME = 4,
	REGEN = 5,
	FREQ = 6,
	RESO = 7,
	FLUTTER = 8,
	DRY_WET = 9
} PortIndex;

typedef struct {
	double sampleRate;
	const float* input[2];
	float* output[2];
	const float* time;
	const float* regen;
	const float* freq;
	const float* reso;
	const float* flutter;
	const float* dryWet;

	double dL[88211];
	double prevSampleL;
	double delayL;
	double sweepL;
	double regenFilterL[9];
	double outFilterL[9];
	double lastRefL[10];

	double dR[88211];
	double prevSampleR;
	double delayR;
	double sweepR;
	double regenFilterR[9];
	double outFilterR[9];
	double lastRefR[10];

	int cycle;

	uint32_t fpdL;
	uint32_t fpdR;
} TapeDelay2;

static LV2_Handle instantiate(
	const LV2_Descriptor* descriptor,
	double rate,
	const char* bundle_path,
	const LV2_Feature* const* features)
{
	TapeDelay2* tapeDelay2 = (TapeDelay2*) calloc(1, sizeof(TapeDelay2));
	tapeDelay2->sampleRate = rate;

	for (int x = 0; x < 88210; x++) {
		tapeDelay2->dL[x] = 0.0;
		tapeDelay2->dR[x] = 0.0;
	}
	tapeDelay2->prevSampleL = 0.0;
	tapeDelay2->delayL = 0.0;
	tapeDelay2->sweepL = 0.0;
	tapeDelay2->prevSampleR = 0.0;
	tapeDelay2->delayR = 0.0;
	tapeDelay2->sweepR = 0.0;
	for (int x = 0; x < 9; x++) {
		tapeDelay2->regenFilterL[x] = 0.0;
		tapeDelay2->outFilterL[x] = 0.0;
		tapeDelay2->lastRefL[x] = 0.0;
		tapeDelay2->regenFilterR[x] = 0.0;
		tapeDelay2->outFilterR[x] = 0.0;
		tapeDelay2->lastRefR[x] = 0.0;
	}
	tapeDelay2->cycle = 0;

	tapeDelay2->fpdL = 1.0;
	while (tapeDelay2->fpdL < 16386) {
		tapeDelay2->fpdL = rand() * UINT32_MAX;
	}
	tapeDelay2->fpdR = 1.0;
	while (tapeDelay2->fpdR < 16386) {
		tapeDelay2->fpdR = rand() * UINT32_MAX;
	}

	return (LV2_Handle) tapeDelay2;
}

static void connect_port(LV2_Handle instance, uint32_t port, void* data)
{
	TapeDelay2* tapeDelay2 = (TapeDelay2*) instance;

	switch ((PortIndex) port) {
		case INPUT_L:
			tapeDelay2->input[0] = (const float*) data;
			break;
		case INPUT_R:
			tapeDelay2->input[1] = (const float*) data;
			break;
		case OUTPUT_L:
			tapeDelay2->output[0] = (float*) data;
			break;
		case OUTPUT_R:
			tapeDelay2->output[1] = (float*) data;
			break;
		case TIME:
			tapeDelay2->time = (const float*) data;
			break;
		case REGEN:
			tapeDelay2->regen = (const float*) data;
			break;
		case FREQ:
			tapeDelay2->freq = (const float*) data;
			break;
		case RESO:
			tapeDelay2->reso = (const float*) data;
			break;
		case FLUTTER:
			tapeDelay2->flutter = (const float*) data;
			break;
		case DRY_WET:
			tapeDelay2->dryWet = (const float*) data;
			break;
		default:
			break;
	}
}

static void activate(LV2_Handle instance)
{
}

static void run(LV2_Handle instance, uint32_t sampleCount)
{
	TapeDelay2* self = (TapeDelay2*) instance;

	const float* in1 = self->input[0];
	const float* in2 = self->input[1];
	float* out1 = self->output[0];
	float* out2 = self->output[1];

	double A = *self->time;
	double B = *self->regen;
	double C = *self->freq;
	double D = *self->reso;
	double E = *self->flutter;
	double F = *self->dryWet;

	double overallscale = 1.0;
	overallscale /= 44100.0;
	overallscale *= self->sampleRate;

	int cycleEnd = floor(overallscale);
	if (cycleEnd < 1) cycleEnd = 1;
	if (cycleEnd > 4) cycleEnd = 4;
	if (self->cycle > cycleEnd - 1) self->cycle = cycleEnd - 1;

	double baseSpeed = (pow(A, 4) * 25.0) + 1.0;
	double feedback = pow(B, 2);

	self->regenFilterL[0] = self->regenFilterR[0] = ((pow(C, 3) * 0.4) + 0.0001);
	self->regenFilterL[1] = self->regenFilterR[1] = pow(D, 2) + 0.01;
	double K = tan(M_PI * self->regenFilterR[0]);
	double norm = 1.0 / (1.0 + K / self->regenFilterR[1] + K * K);
	self->regenFilterL[2] = self->regenFilterR[2] = K / self->regenFilterR[1] * norm;
	self->regenFilterL[4] = self->regenFilterR[4] = -self->regenFilterR[2];
	self->regenFilterL[5] = self->regenFilterR[5] = 2.0 * (K * K - 1.0) * norm;
	self->regenFilterL[6] = self->regenFilterR[6] = (1.0 - K / self->regenFilterR[1] + K * K) * norm;

	self->outFilterL[0] = self->outFilterR[0] = self->regenFilterR[0];
	self->outFilterL[1] = self->outFilterR[1] = self->regenFilterR[1] * 1.618033988749894848204586;
	K = tan(M_PI * self->outFilterR[0]);
	norm = 1.0 / (1.0 + K / self->outFilterR[1] + K * K);
	self->outFilterL[2] = self->outFilterR[2] = K / self->outFilterR[1] * norm;
	self->outFilterL[4] = self->outFilterR[4] = -self->outFilterR[2];
	self->outFilterL[5] = self->outFilterR[5] = 2.0 * (K * K - 1.0) * norm;
	self->outFilterL[6] = self->outFilterR[6] = (1.0 - K / self->outFilterR[1] + K * K) * norm;

	double vibSpeed = pow(E, 5) * baseSpeed * ((self->regenFilterR[0] * 0.09) + 0.025);
	double wet = F * 2.0;
	double dry = 2.0 - wet;
	if (wet > 1.0) wet = 1.0;
	if (wet < 0.0) wet = 0.0;
	if (dry > 1.0) dry = 1.0;
	if (dry < 0.0) dry = 0.0;

	for (uint32_t i = 0; i < sampleCount; i++) {
		double inputSampleL = in1[i];
		double inputSampleR = in2[i];
		if (fabs(inputSampleL) < 1.18e-23) inputSampleL = self->fpdL * 1.18e-17;
		if (fabs(inputSampleR) < 1.18e-23) inputSampleR = self->fpdR * 1.18e-17;
		double drySampleL = inputSampleL;
		double drySampleR = inputSampleR;

		self->cycle++;
		if (self->cycle == cycleEnd) {
			double speedL = baseSpeed + (vibSpeed * (sin(self->sweepL) + 1.0));
			double speedR = baseSpeed + (vibSpeed * (sin(self->sweepR) + 1.0));
			self->sweepL += (0.05 * inputSampleL * inputSampleL);
			if (self->sweepL > 6.283185307179586) self->sweepL -= 6.283185307179586;
			self->sweepR += (0.05 * inputSampleR * inputSampleR);
			if (self->sweepR > 6.283185307179586) self->sweepR -= 6.283185307179586;

			// begin left channel
			int pos = floor(self->delayL);
			double newSample = inputSampleL + self->dL[pos] * feedback;
			double tempSample = (newSample * self->regenFilterL[2]) + self->regenFilterL[7];
			self->regenFilterL[7] = -(tempSample * self->regenFilterL[5]) + self->regenFilterL[8];
			self->regenFilterL[8] = (newSample * self->regenFilterL[4]) - (tempSample * self->regenFilterL[6]);
			newSample = tempSample;

			self->delayL -= speedL;
			if (self->delayL < 0) self->delayL += 88200.0;
			double increment = (newSample - self->prevSampleL) / speedL;
			self->dL[pos] = self->prevSampleL;
			while (pos != floor(self->delayL)) {
				self->dL[pos] = self->prevSampleL;
				self->prevSampleL += increment;
				pos--;
				if (pos < 0) pos += 88200;
			}
			self->prevSampleL = newSample;
			pos = floor(self->delayL);
			inputSampleL = self->dL[pos];
			tempSample = (inputSampleL * self->outFilterL[2]) + self->outFilterL[7];
			self->outFilterL[7] = -(tempSample * self->outFilterL[5]) + self->outFilterL[8];
			self->outFilterL[8] = (inputSampleL * self->outFilterL[4]) - (tempSample * self->outFilterL[6]);
			inputSampleL = tempSample;
			// end left channel

			// begin right channel
			pos = floor(self->delayR);
			newSample = inputSampleR + self->dR[pos] * feedback;
			tempSample = (newSample * self->regenFilterR[2]) + self->regenFilterR[7];
			self->regenFilterR[7] = -(tempSample * self->regenFilterR[5]) + self->regenFilterR[8];
			self->regenFilterR[8] = (newSample * self->regenFilterR[4]) - (tempSample * self->regenFilterR[6]);
			newSample = tempSample;

			self->delayR -= speedR;
			if (self->delayR < 0) self->delayR += 88200.0;
			increment = (newSample - self->prevSampleR) / speedR;
			self->dR[pos] = self->prevSampleR;
			while (pos != floor(self->delayR)) {
				self->dR[pos] = self->prevSampleR;
				self->prevSampleR += increment;
				pos--;
				if (pos < 0) pos += 88200;
			}
			self->prevSampleR = newSample;
			pos = floor(self->delayR);
			inputSampleR = self->dR[pos];
			tempSample = (inputSampleR * self->outFilterR[2]) + self->outFilterR[7];
			self->outFilterR[7] = -(tempSample * self->outFilterR[5]) + self->outFilterR[8];
			self->outFilterR[8] = (inputSampleR * self->outFilterR[4]) - (tempSample * self->outFilterR[6]);
			inputSampleR = tempSample;
			// end right channel

			if (cycleEnd == 4) {
				self->lastRefL[0] = self->lastRefL[4];
				self->lastRefL[2] = (self->lastRefL[0] + inputSampleL) / 2;
				self->lastRefL[1] = (self->lastRefL[0] + self->lastRefL[2]) / 2;
				self->lastRefL[3] = (self->lastRefL[2] + inputSampleL) / 2;
				self->lastRefL[4] = inputSampleL;
				self->lastRefR[0] = self->lastRefR[4];
				self->lastRefR[2] = (self->lastRefR[0] + inputSampleR) / 2;
				self->lastRefR[1] = (self->lastRefR[0] + self->lastRefR[2]) / 2;
				self->lastRefR[3] = (self->lastRefR[2] + inputSampleR) / 2;
				self->lastRefR[4] = inputSampleR;
			}
			if (cycleEnd == 3) {
				self->lastRefL[0] = self->lastRefL[3];
				self->lastRefL[2] = (self->lastRefL[0] + self->lastRefL[0] + inputSampleL) / 3;
				self->lastRefL[1] = (self->lastRefL[0] + inputSampleL + inputSampleL) / 3;
				self->lastRefL[3] = inputSampleL;
				self->lastRefR[0] = self->lastRefR[3];
				self->lastRefR[2] = (self->lastRefR[0] + self->lastRefR[0] + inputSampleR) / 3;
				self->lastRefR[1] = (self->lastRefR[0] + inputSampleR + inputSampleR) / 3;
				self->lastRefR[3] = inputSampleR;
			}
			if (cycleEnd == 2) {
				self->lastRefL[0] = self->lastRefL[2];
				self->lastRefL[1] = (self->lastRefL[0] + inputSampleL) / 2;
				self->lastRefL[2] = inputSampleL;
				self->lastRefR[0] = self->lastRefR[2];
				self->lastRefR[1] = (self->lastRefR[0] + inputSampleR) / 2;
				self->lastRefR[2] = inputSampleR;
			}
			if (cycleEnd == 1) {
				self->lastRefL[0] = inputSampleL;
				self->lastRefR[0] = inputSampleR;
			}
			self->cycle = 0;
			inputSampleL = self->lastRefL[self->cycle];
			inputSampleR = self->lastRefR[self->cycle];
		} else {
			inputSampleL = self->lastRefL[self->cycle];
			inputSampleR = self->lastRefR[self->cycle];
		}

		switch (cycleEnd) {
			case 4:
				self->lastRefL[8] = inputSampleL;
				inputSampleL = (inputSampleL + self->lastRefL[7]) * 0.5;
				self->lastRefL[7] = self->lastRefL[8];
				self->lastRefR[8] = inputSampleR;
				inputSampleR = (inputSampleR + self->lastRefR[7]) * 0.5;
				self->lastRefR[7] = self->lastRefR[8];
				// fall through
			case 3:
				self->lastRefL[8] = inputSampleL;
				inputSampleL = (inputSampleL + self->lastRefL[6]) * 0.5;
				self->lastRefL[6] = self->lastRefL[8];
				self->lastRefR[8] = inputSampleR;
				inputSampleR = (inputSampleR + self->lastRefR[6]) * 0.5;
				self->lastRefR[6] = self->lastRefR[8];
				// fall through
			case 2:
				self->lastRefL[8] = inputSampleL;
				inputSampleL = (inputSampleL + self->lastRefL[5]) * 0.5;
				self->lastRefL[5] = self->lastRefL[8];
				self->lastRefR[8] = inputSampleR;
				inputSampleR = (inputSampleR + self->lastRefR[5]) * 0.5;
				self->lastRefR[5] = self->lastRefR[8];
				// fall through
			case 1:
				break;
		}

		if (wet < 1.0) {
			inputSampleL *= wet;
			inputSampleR *= wet;
		}
		if (dry < 1.0) {
			drySampleL *= dry;
			drySampleR *= dry;
		}
		inputSampleL += drySampleL;
		inputSampleR += drySampleR;

		// begin 32 bit stereo floating point dither
		int expon;
		frexpf((float)inputSampleL, &expon);
		self->fpdL ^= self->fpdL << 13;
		self->fpdL ^= self->fpdL >> 17;
		self->fpdL ^= self->fpdL << 5;
		inputSampleL += ((double(self->fpdL) - uint32_t(0x7fffffff)) * 5.5e-36l * pow(2, expon + 62));
		frexpf((float)inputSampleR, &expon);
		self->fpdR ^= self->fpdR << 13;
		self->fpdR ^= self->fpdR >> 17;
		self->fpdR ^= self->fpdR << 5;
		inputSampleR += ((double(self->fpdR) - uint32_t(0x7fffffff)) * 5.5e-36l * pow(2, expon + 62));
		// end 32 bit stereo floating point dither

		out1[i] = inputSampleL;
		out2[i] = inputSampleR;
	}
}

static void deactivate(LV2_Handle instance)
{
}

static void cleanup(LV2_Handle instance)
{
	free(instance);
}

static const void* extension_data(const char* uri)
{
	return NULL;
}

static const LV2_Descriptor descriptor = {
	TAPEDELAY2_URI,
	instantiate,
	connect_port,
	activate,
	run,
	deactivate,
	cleanup,
	extension_data
};

LV2_SYMBOL_EXPORT const LV2_Descriptor* lv2_descriptor(uint32_t index)
{
	return index == 0 ? &descriptor : NULL;
}
