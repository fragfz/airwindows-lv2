#include <lv2/core/lv2.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define SINGLEENDEDTRIODE_URI "https://hannesbraun.net/ns/lv2/airwindows/singleendedtriode"

typedef enum {
	INPUT_L = 0,
	OUTPUT_L = 1,
	TRIODE = 2,
	CLASS_AB = 3,
	CLASS_B = 4,
	DRY_WET = 5,
	BIAS = 6
} PortIndex;

typedef struct {
	const float* input;
	float* output;
	const float* triode;
	const float* classAB;
	const float* classB;
	const float* dryWet;
	const float* bias;
	uint32_t fpdL;
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
			singleEndedTriode->input = (const float*) data;
			break;
		case OUTPUT_L:
			singleEndedTriode->output = (float*) data;
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
		case BIAS:
			singleEndedTriode->bias = (const float*) data;
			break;
	}
}

static void activate(LV2_Handle instance)
{
	SingleEndedTriode* singleEndedTriode = (SingleEndedTriode*) instance;
	singleEndedTriode->fpdL = 1.0;
	while (singleEndedTriode->fpdL < 16386) singleEndedTriode->fpdL = rand() * UINT32_MAX;
}

static void run(LV2_Handle instance, uint32_t sampleFrames)
{
	SingleEndedTriode* singleEndedTriode = (SingleEndedTriode*) instance;

	const float* in1 = singleEndedTriode->input;
	float* out1 = singleEndedTriode->output;
	double intensity = pow(*singleEndedTriode->triode, 2) * 8.0;
	double triode = intensity;
	intensity += 0.001;
	double softcrossover = pow(*singleEndedTriode->classAB, 3) / 8.0;
	double hardcrossover = pow(*singleEndedTriode->classB, 7) / 8.0;
	double wet = *singleEndedTriode->dryWet;
	double bias = *singleEndedTriode->bias;
	double postsine = sin(bias);

	while (sampleFrames-- > 0) {
		double inputSampleL = *in1;
		if (fabs(inputSampleL) < 1.18e-23) inputSampleL = singleEndedTriode->fpdL * 1.18e-17;
		double drySampleL = inputSampleL;

		if (triode > 0.0) {
			inputSampleL *= intensity;
			inputSampleL -= bias;

			double bridgerectifier = fabs(inputSampleL);
			if (bridgerectifier > 1.57079633) bridgerectifier = 1.57079633;
			bridgerectifier = sin(bridgerectifier);
			if (inputSampleL > 0) inputSampleL = bridgerectifier;
			else inputSampleL = -bridgerectifier;

			inputSampleL += postsine;
			inputSampleL /= intensity;
		}

		if (softcrossover > 0.0) {
			double bridgerectifier = fabs(inputSampleL);
			if (bridgerectifier > 0.0) bridgerectifier -= (softcrossover * (bridgerectifier + sqrt(bridgerectifier)));
			if (bridgerectifier < 0.0) bridgerectifier = 0;
			if (inputSampleL > 0.0) inputSampleL = bridgerectifier;
			else inputSampleL = -bridgerectifier;
		}

		if (hardcrossover > 0.0) {
			double bridgerectifier = fabs(inputSampleL);
			bridgerectifier -= hardcrossover;
			if (bridgerectifier < 0.0) bridgerectifier = 0.0;
			if (inputSampleL > 0.0) inputSampleL = bridgerectifier;
			else inputSampleL = -bridgerectifier;
		}

		if (wet != 1.0) {
			inputSampleL = (inputSampleL * wet) + (drySampleL * (1.0 - wet));
		}

		int expon;
		frexpf((float) inputSampleL, &expon);
		singleEndedTriode->fpdL ^= singleEndedTriode->fpdL << 13;
		singleEndedTriode->fpdL ^= singleEndedTriode->fpdL >> 17;
		singleEndedTriode->fpdL ^= singleEndedTriode->fpdL << 5;
		inputSampleL += (((double) singleEndedTriode->fpdL - (uint32_t) 0x7fffffff) * 5.5e-36l * pow(2, expon + 62));

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
