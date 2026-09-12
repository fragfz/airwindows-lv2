#include <lv2/core/lv2.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define TAPEDELAY2_URI "https://hannesbraun.net/ns/lv2/airwindows/tapedelay2"

typedef enum {
	INPUT_L = 0,
	OUTPUT_L = 1,
	TIME = 2,
	REGEN = 3,
	FREQ = 4,
	RESO = 5,
	FLUTTER = 6,
	DRY_WET = 7
} PortIndex;

typedef struct {
	double sampleRate;
	const float* input;
	float* output;
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

	int cycle;

	uint32_t fpdL;
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
	}
	tapeDelay2->prevSampleL = 0.0;
	tapeDelay2->delayL = 0.0;
	tapeDelay2->sweepL = 0.0;
	for (int x = 0; x < 9; x++) {
		tapeDelay2->regenFilterL[x] = 0.0;
		tapeDelay2->outFilterL[x] = 0.0;
		tapeDelay2->lastRefL[x] = 0.0;
	}
	tapeDelay2->cycle = 0;

	tapeDelay2->fpdL = 1.0;
	while (tapeDelay2->fpdL < 16386) {
		tapeDelay2->fpdL = rand() * UINT32_MAX;
	}

	return (LV2_Handle) tapeDelay2;
}

static void connect_port(LV2_Handle instance, uint32_t port, void* data)
{
	TapeDelay2* tapeDelay2 = (TapeDelay2*) instance;

	switch ((PortIndex) port) {
		case INPUT_L:
			tapeDelay2->input = (const float*) data;
			break;
		case OUTPUT_L:
			tapeDelay2->output = (float*) data;
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

	const float* in1 = self->input;
	float* out1 = self->output;

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

	self->regenFilterL[0] = ((pow(C, 3) * 0.4) + 0.0001);
	self->regenFilterL[1] = pow(D, 2) + 0.01;
	double K = tan(M_PI * self->regenFilterL[0]);
	double norm = 1.0 / (1.0 + K / self->regenFilterL[1] + K * K);
	self->regenFilterL[2] = K / self->regenFilterL[1] * norm;
	self->regenFilterL[4] = -self->regenFilterL[2];
	self->regenFilterL[5] = 2.0 * (K * K - 1.0) * norm;
	self->regenFilterL[6] = (1.0 - K / self->regenFilterL[1] + K * K) * norm;

	self->outFilterL[0] = self->regenFilterL[0];
	self->outFilterL[1] = self->regenFilterL[1] * 1.618033988749894848204586;
	K = tan(M_PI * self->outFilterL[0]);
	norm = 1.0 / (1.0 + K / self->outFilterL[1] + K * K);
	self->outFilterL[2] = K / self->outFilterL[1] * norm;
	self->outFilterL[4] = -self->outFilterL[2];
	self->outFilterL[5] = 2.0 * (K * K - 1.0) * norm;
	self->outFilterL[6] = (1.0 - K / self->outFilterL[1] + K * K) * norm;

	double vibSpeed = pow(E, 5) * baseSpeed * ((self->regenFilterL[0] * 0.09) + 0.025);
	double wet = F * 2.0;
	double dry = 2.0 - wet;
	if (wet > 1.0) wet = 1.0;
	if (wet < 0.0) wet = 0.0;
	if (dry > 1.0) dry = 1.0;
	if (dry < 0.0) dry = 0.0;

	for (uint32_t i = 0; i < sampleCount; i++) {
		double inputSampleL = in1[i];
		if (fabs(inputSampleL) < 1.18e-23) inputSampleL = self->fpdL * 1.18e-17;
		double drySampleL = inputSampleL;

		self->cycle++;
		if (self->cycle == cycleEnd) {
			double speedL = baseSpeed + (vibSpeed * (sin(self->sweepL) + 1.0));
			self->sweepL += (0.05 * inputSampleL * inputSampleL);
			if (self->sweepL > 6.283185307179586) self->sweepL -= 6.283185307179586;

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

			if (cycleEnd == 4) {
				self->lastRefL[0] = self->lastRefL[4];
				self->lastRefL[2] = (self->lastRefL[0] + inputSampleL) / 2;
				self->lastRefL[1] = (self->lastRefL[0] + self->lastRefL[2]) / 2;
				self->lastRefL[3] = (self->lastRefL[2] + inputSampleL) / 2;
				self->lastRefL[4] = inputSampleL;
			}
			if (cycleEnd == 3) {
				self->lastRefL[0] = self->lastRefL[3];
				self->lastRefL[2] = (self->lastRefL[0] + self->lastRefL[0] + inputSampleL) / 3;
				self->lastRefL[1] = (self->lastRefL[0] + inputSampleL + inputSampleL) / 3;
				self->lastRefL[3] = inputSampleL;
			}
			if (cycleEnd == 2) {
				self->lastRefL[0] = self->lastRefL[2];
				self->lastRefL[1] = (self->lastRefL[0] + inputSampleL) / 2;
				self->lastRefL[2] = inputSampleL;
			}
			if (cycleEnd == 1) {
				self->lastRefL[0] = inputSampleL;
			}
			self->cycle = 0;
			inputSampleL = self->lastRefL[self->cycle];
		} else {
			inputSampleL = self->lastRefL[self->cycle];
		}

		switch (cycleEnd) {
			case 4:
				self->lastRefL[8] = inputSampleL;
				inputSampleL = (inputSampleL + self->lastRefL[7]) * 0.5;
				self->lastRefL[7] = self->lastRefL[8];
				// fall through
			case 3:
				self->lastRefL[8] = inputSampleL;
				inputSampleL = (inputSampleL + self->lastRefL[6]) * 0.5;
				self->lastRefL[6] = self->lastRefL[8];
				// fall through
			case 2:
				self->lastRefL[8] = inputSampleL;
				inputSampleL = (inputSampleL + self->lastRefL[5]) * 0.5;
				self->lastRefL[5] = self->lastRefL[8];
				// fall through
			case 1:
				break;
		}

		if (wet < 1.0) {
			inputSampleL *= wet;
		}
		if (dry < 1.0) {
			drySampleL *= dry;
		}
		inputSampleL += drySampleL;

		// begin 32 bit stereo floating point dither
		int expon;
		frexpf((float)inputSampleL, &expon);
		self->fpdL ^= self->fpdL << 13;
		self->fpdL ^= self->fpdL >> 17;
		self->fpdL ^= self->fpdL << 5;
		inputSampleL += ((double(self->fpdL) - uint32_t(0x7fffffff)) * 5.5e-36l * pow(2, expon + 62));
		// end 32 bit stereo floating point dither

		out1[i] = inputSampleL;
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
