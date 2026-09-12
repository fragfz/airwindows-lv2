#include <lv2/core/lv2.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define TAPEHACK2_URI "https://hannesbraun.net/ns/lv2/airwindows/tapehack2"

typedef enum {
	INPUT_L = 0,
	OUTPUT_L = 1,
	INPUT = 2,
	OUTPUT = 3,
	DRY_WET = 4
} PortIndex;

typedef struct {
	double sampleRate;
	const float* input;
	float* output;
	const float* inputGain;
	const float* outputGain;
	const float* dryWet;

	double avg32L[33];
	double avg16L[17];
	double avg8L[9];
	double avg4L[5];
	double avg2L[3];
	double post32L[33];
	double post16L[17];
	double post8L[9];
	double post4L[5];
	double post2L[3];
	double lastDarkL;
	int avgPos;

	uint32_t fpdL;
} TapeHack2;

static LV2_Handle instantiate(
	const LV2_Descriptor* descriptor,
	double rate,
	const char* bundle_path,
	const LV2_Feature* const* features)
{
	TapeHack2* tapeHack2 = (TapeHack2*) calloc(1, sizeof(TapeHack2));
	tapeHack2->sampleRate = rate;
	return (LV2_Handle) tapeHack2;
}

static void connect_port(LV2_Handle instance, uint32_t port, void* data)
{
	TapeHack2* tapeHack2 = (TapeHack2*) instance;

	switch ((PortIndex) port) {
		case INPUT_L:
			tapeHack2->input = (const float*) data;
			break;
		case OUTPUT_L:
			tapeHack2->output = (float*) data;
			break;
		case INPUT:
			tapeHack2->inputGain = (const float*) data;
			break;
		case OUTPUT:
			tapeHack2->outputGain = (const float*) data;
			break;
		case DRY_WET:
			tapeHack2->dryWet = (const float*) data;
			break;
	}
}

static void activate(LV2_Handle instance)
{
	TapeHack2* tapeHack2 = (TapeHack2*) instance;

	for (int x = 0; x < 33; x++) {
		tapeHack2->avg32L[x] = 0.0;
		tapeHack2->post32L[x] = 0.0;
	}
	for (int x = 0; x < 17; x++) {
		tapeHack2->avg16L[x] = 0.0;
		tapeHack2->post16L[x] = 0.0;
	}
	for (int x = 0; x < 9; x++) {
		tapeHack2->avg8L[x] = 0.0;
		tapeHack2->post8L[x] = 0.0;
	}
	for (int x = 0; x < 5; x++) {
		tapeHack2->avg4L[x] = 0.0;
		tapeHack2->post4L[x] = 0.0;
	}
	for (int x = 0; x < 3; x++) {
		tapeHack2->avg2L[x] = 0.0;
		tapeHack2->post2L[x] = 0.0;
	}
	tapeHack2->avgPos = 0;
	tapeHack2->lastDarkL = 0.0;

	tapeHack2->fpdL = 1.0;
	while (tapeHack2->fpdL < 16386) tapeHack2->fpdL = rand() * UINT32_MAX;
}

static void run(LV2_Handle instance, uint32_t sampleFrames)
{
	TapeHack2* tapeHack2 = (TapeHack2*) instance;

	const float* in1 = tapeHack2->input;
	float* out1 = tapeHack2->output;

	double overallscale = 1.0;
	overallscale /= 44100.0;
	overallscale *= tapeHack2->sampleRate;
	int spacing = (int) floor(overallscale * 2.0);
	if (spacing < 2) spacing = 2;
	if (spacing > 32) spacing = 32;

	double inputGain = *tapeHack2->inputGain * 10.0;
	double outputGain = *tapeHack2->outputGain * 0.9239;
	double wet = *tapeHack2->dryWet;

	while (sampleFrames-- > 0) {
		double inputSampleL = *in1;
		if (fabs(inputSampleL) < 1.18e-23) inputSampleL = tapeHack2->fpdL * 1.18e-17;
		double drySampleL = inputSampleL;

		inputSampleL *= inputGain;
		double darkSampleL = inputSampleL;
		if (tapeHack2->avgPos > 31) tapeHack2->avgPos = 0;
		if (spacing > 31) {
			tapeHack2->avg32L[tapeHack2->avgPos] = darkSampleL;
			darkSampleL = 0.0;
			for (int x = 0; x < 32; x++) {
				darkSampleL += tapeHack2->avg32L[x];
			}
			darkSampleL /= 32.0;
		}
		if (spacing > 15) {
			tapeHack2->avg16L[tapeHack2->avgPos % 16] = darkSampleL;
			darkSampleL = 0.0;
			for (int x = 0; x < 16; x++) {
				darkSampleL += tapeHack2->avg16L[x];
			}
			darkSampleL /= 16.0;
		}
		if (spacing > 7) {
			tapeHack2->avg8L[tapeHack2->avgPos % 8] = darkSampleL;
			darkSampleL = 0.0;
			for (int x = 0; x < 8; x++) {
				darkSampleL += tapeHack2->avg8L[x];
			}
			darkSampleL /= 8.0;
		}
		if (spacing > 3) {
			tapeHack2->avg4L[tapeHack2->avgPos % 4] = darkSampleL;
			darkSampleL = 0.0;
			for (int x = 0; x < 4; x++) {
				darkSampleL += tapeHack2->avg4L[x];
			}
			darkSampleL /= 4.0;
		}
		if (spacing > 1) {
			tapeHack2->avg2L[tapeHack2->avgPos % 2] = darkSampleL;
			darkSampleL = 0.0;
			for (int x = 0; x < 2; x++) {
				darkSampleL += tapeHack2->avg2L[x];
			}
			darkSampleL /= 2.0;
		}

		double avgSlewL = fmin(fabs(tapeHack2->lastDarkL - inputSampleL) * 0.12 * overallscale, 1.0);
		avgSlewL = 1.0 - (1.0 - avgSlewL * 1.0 - avgSlewL);
		inputSampleL = (inputSampleL * (1.0 - avgSlewL)) + (darkSampleL * avgSlewL);
		tapeHack2->lastDarkL = darkSampleL;

		inputSampleL = fmax(fmin(inputSampleL, 2.305929007734908), -2.305929007734908);
		double addtwo = inputSampleL * inputSampleL;
		double empower = inputSampleL * addtwo;
		inputSampleL -= (empower / 6.0);
		empower *= addtwo;
		inputSampleL += (empower / 69.0);
		empower *= addtwo;
		inputSampleL -= (empower / 2530.08);
		empower *= addtwo;
		inputSampleL += (empower / 224985.6);
		empower *= addtwo;
		inputSampleL -= (empower / 9979200.0f);

		darkSampleL = inputSampleL;
		if (tapeHack2->avgPos > 31) tapeHack2->avgPos = 0;
		if (spacing > 31) {
			tapeHack2->post32L[tapeHack2->avgPos] = darkSampleL;
			darkSampleL = 0.0;
			for (int x = 0; x < 32; x++) {
				darkSampleL += tapeHack2->post32L[x];
			}
			darkSampleL /= 32.0;
		}
		if (spacing > 15) {
			tapeHack2->post16L[tapeHack2->avgPos % 16] = darkSampleL;
			darkSampleL = 0.0;
			for (int x = 0; x < 16; x++) {
				darkSampleL += tapeHack2->post16L[x];
			}
			darkSampleL /= 16.0;
		}
		if (spacing > 7) {
			tapeHack2->post8L[tapeHack2->avgPos % 8] = darkSampleL;
			darkSampleL = 0.0;
			for (int x = 0; x < 8; x++) {
				darkSampleL += tapeHack2->post8L[x];
			}
			darkSampleL /= 8.0;
		}
		if (spacing > 3) {
			tapeHack2->post4L[tapeHack2->avgPos % 4] = darkSampleL;
			darkSampleL = 0.0;
			for (int x = 0; x < 4; x++) {
				darkSampleL += tapeHack2->post4L[x];
			}
			darkSampleL /= 4.0;
		}
		if (spacing > 1) {
			tapeHack2->post2L[tapeHack2->avgPos % 2] = darkSampleL;
			darkSampleL = 0.0;
			for (int x = 0; x < 2; x++) {
				darkSampleL += tapeHack2->post2L[x];
			}
			darkSampleL /= 2.0;
		}
		tapeHack2->avgPos++;
		inputSampleL = (inputSampleL * (1.0 - avgSlewL)) + (darkSampleL * avgSlewL);

		inputSampleL = (inputSampleL * outputGain * wet) + (drySampleL * (1.0 - wet));

		int expon;
		frexpf((float) inputSampleL, &expon);
		tapeHack2->fpdL ^= tapeHack2->fpdL << 13;
		tapeHack2->fpdL ^= tapeHack2->fpdL >> 17;
		tapeHack2->fpdL ^= tapeHack2->fpdL << 5;
		inputSampleL += (((double) tapeHack2->fpdL - (uint32_t) 0x7fffffff) * 5.5e-36l * pow(2, expon + 62));

		*out1 = (float) inputSampleL;

		in1++;
		out1++;
	}
}

static void deactivate(LV2_Handle instance) {}

static void cleanup(LV2_Handle instance)
{
	free(instance);
}

static const void* extension_data(const char* uri)
{
	return NULL;
}

static const LV2_Descriptor descriptor = {
	TAPEHACK2_URI,
	instantiate,
	connect_port,
	activate,
	run,
	deactivate,
	cleanup,
	extension_data};

LV2_SYMBOL_EXPORT const LV2_Descriptor* lv2_descriptor(uint32_t index)
{
	return index == 0 ? &descriptor : NULL;
}
