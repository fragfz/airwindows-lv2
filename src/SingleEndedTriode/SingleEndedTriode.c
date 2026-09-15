#include <lv2/core/lv2.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define SINGLEENDEDTRIODE_URI "https://hannesbraun.net/ns/lv2/airwindows/singleendedtriode"

typedef enum {
	INPUT_L = 0,
	INPUT_R = 1,
	OUTPUT_L = 2,
	OUTPUT_R = 3,
	TRIODE = 4,
	CLASS_AB = 5,
	CLASS_B = 6,
	DRY_WET = 7
} PortIndex;

typedef struct {
	const float* input[2];
	float* output[2];
	const float* triode;
	const float* classAB;
	const float* classB;
	const float* dryWet;

	double postsine;
	uint32_t fpdL;
	uint32_t fpdR;
} SingleEndedTriode;

static LV2_Handle instantiate(
	const LV2_Descriptor* descriptor,
	double rate,
	const char* bundle_path,
	const LV2_Feature* const* features)
{
	SingleEndedTriode* singleEndedTriode = (SingleEndedTriode*) calloc(1, sizeof(SingleEndedTriode));
	return (LV2_Handle) singleEndedTriode;
}

static void connect_port(LV2_Handle instance, uint32_t port, void* data)
{
	SingleEndedTriode* singleEndedTriode = (SingleEndedTriode*) instance;

	switch ((PortIndex) port) {
		case INPUT_L:
			singleEndedTriode->input[0] = (const float*) data;
			break;
		case INPUT_R:
			singleEndedTriode->input[1] = (const float*) data;
			break;
		case OUTPUT_L:
			singleEndedTriode->output[0] = (float*) data;
			break;
		case OUTPUT_R:
			singleEndedTriode->output[1] = (float*) data;
			break;
		case TRIODE:
			singleEndedTriode->triode = (const float*) data;
			break;
		case CLASS_AB:
			singleEndedTriode->classAB = (const float*) data;
			break;
		case CLASS_B:
			singleEndedTriode->classB = (const float*) data;
			break;
		case DRY_WET:
			singleEndedTriode->dryWet = (const float*) data;
			break;
	}
}

static void activate(LV2_Handle instance)
{
	SingleEndedTriode* singleEndedTriode = (SingleEndedTriode*) instance;
	singleEndedTriode->postsine = sin(0.5);

	singleEndedTriode->fpdL = 1.0;
	while (singleEndedTriode->fpdL < 16386) singleEndedTriode->fpdL = rand() * UINT32_MAX;
	singleEndedTriode->fpdR = 1.0;
	while (singleEndedTriode->fpdR < 16386) singleEndedTriode->fpdR = rand() * UINT32_MAX;
}

static void run(LV2_Handle instance, uint32_t sampleFrames)
{
	SingleEndedTriode* singleEndedTriode = (SingleEndedTriode*) instance;

	const float* in1 = singleEndedTriode->input[0];
	const float* in2 = singleEndedTriode->input[1];
	float* out1 = singleEndedTriode->output[0];
	float* out2 = singleEndedTriode->output[1];
	double intensity = pow(*singleEndedTriode->triode, 2) * 8.0;
	double triode = intensity;
	intensity += 0.001;
	double softcrossover = pow(*singleEndedTriode->classAB, 3) / 8.0;
	double hardcrossover = pow(*singleEndedTriode->classB, 7) / 8.0;
	double wet = *singleEndedTriode->dryWet;

	while (sampleFrames-- > 0) {
		double inputSampleL = *in1;
		double inputSampleR = *in2;
		if (fabs(inputSampleL) < 1.18e-23) inputSampleL = singleEndedTriode->fpdL * 1.18e-17;
		if (fabs(inputSampleR) < 1.18e-23) inputSampleR = singleEndedTriode->fpdR * 1.18e-17;
		double drySampleL = inputSampleL;
		double drySampleR = inputSampleR;

		if (triode > 0.0) {
			inputSampleL *= intensity;
			inputSampleR *= intensity;
			inputSampleL -= 0.5;
			inputSampleR -= 0.5;

			double bridgerectifier = fabs(inputSampleL);
			if (bridgerectifier > 1.57079633) bridgerectifier = 1.57079633;
			bridgerectifier = sin(bridgerectifier);
			if (inputSampleL > 0) inputSampleL = bridgerectifier;
			else inputSampleL = -bridgerectifier;

			bridgerectifier = fabs(inputSampleR);
			if (bridgerectifier > 1.57079633) bridgerectifier = 1.57079633;
			bridgerectifier = sin(bridgerectifier);
			if (inputSampleR > 0) inputSampleR = bridgerectifier;
			else inputSampleR = -bridgerectifier;

			inputSampleL += singleEndedTriode->postsine;
			inputSampleR += singleEndedTriode->postsine;
			inputSampleL /= intensity;
			inputSampleR /= intensity;
		}

		if (softcrossover > 0.0) {
			double bridgerectifier = fabs(inputSampleL);
			if (bridgerectifier > 0.0) bridgerectifier -= (softcrossover * (bridgerectifier + sqrt(bridgerectifier)));
			if (bridgerectifier < 0.0) bridgerectifier = 0;
			if (inputSampleL > 0.0) inputSampleL = bridgerectifier;
			else inputSampleL = -bridgerectifier;

			bridgerectifier = fabs(inputSampleR);
			if (bridgerectifier > 0.0) bridgerectifier -= (softcrossover * (bridgerectifier + sqrt(bridgerectifier)));
			if (bridgerectifier < 0.0) bridgerectifier = 0;
			if (inputSampleR > 0.0) inputSampleR = bridgerectifier;
			else inputSampleR = -bridgerectifier;
		}

		if (hardcrossover > 0.0) {
			double bridgerectifier = fabs(inputSampleL);
			bridgerectifier -= hardcrossover;
			if (bridgerectifier < 0.0) bridgerectifier = 0.0;
			if (inputSampleL > 0.0) inputSampleL = bridgerectifier;
			else inputSampleL = -bridgerectifier;

			bridgerectifier = fabs(inputSampleR);
			bridgerectifier -= hardcrossover;
			if (bridgerectifier < 0.0) bridgerectifier = 0.0;
			if (inputSampleR > 0.0) inputSampleR = bridgerectifier;
			else inputSampleR = -bridgerectifier;
		}

		if (wet != 1.0) {
			inputSampleL = (inputSampleL * wet) + (drySampleL * (1.0 - wet));
			inputSampleR = (inputSampleR * wet) + (drySampleR * (1.0 - wet));
		}

		int expon;
		frexpf((float) inputSampleL, &expon);
		singleEndedTriode->fpdL ^= singleEndedTriode->fpdL << 13;
		singleEndedTriode->fpdL ^= singleEndedTriode->fpdL >> 17;
		singleEndedTriode->fpdL ^= singleEndedTriode->fpdL << 5;
		inputSampleL += (((double) singleEndedTriode->fpdL - (uint32_t) 0x7fffffff) * 5.5e-36l * pow(2, expon + 62));
		frexpf((float) inputSampleR, &expon);
		singleEndedTriode->fpdR ^= singleEndedTriode->fpdR << 13;
		singleEndedTriode->fpdR ^= singleEndedTriode->fpdR >> 17;
		singleEndedTriode->fpdR ^= singleEndedTriode->fpdR << 5;
		inputSampleR += (((double) singleEndedTriode->fpdR - (uint32_t) 0x7fffffff) * 5.5e-36l * pow(2, expon + 62));

		*out1 = (float) inputSampleL;
		*out2 = (float) inputSampleR;

		in1++;
		in2++;
		out1++;
		out2++;
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
	SINGLEENDEDTRIODE_URI,
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
