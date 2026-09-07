#include <lv2/core/lv2.h>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define TREMOLO_URI "https://hannesbraun.net/ns/lv2/airwindows/tremolo"

typedef enum {
	INPUT_L = 0,
	INPUT_R = 1,
	OUTPUT_L = 2,
	OUTPUT_R = 3,
	SPEED = 4,
	DEPTH = 5
} PortIndex;

typedef struct {
	double sampleRate;
	const float* input[2];
	float* output[2];
	const float* speed;
	const float* depth;

	uint32_t fpdL;
	uint32_t fpdR;
	double sweep;
	double speedChase;
	double depthChase;
	double speedAmount;
	double depthAmount;
	double lastSpeed;
	double lastDepth;
} Tremolo;

static LV2_Handle instantiate(
	const LV2_Descriptor* descriptor,
	double rate,
	const char* bundle_path,
	const LV2_Feature* const* features)
{
	Tremolo* tremolo = (Tremolo*) calloc(1, sizeof(Tremolo));
	tremolo->sampleRate = rate;
	return (LV2_Handle) tremolo;
}

static void connect_port(LV2_Handle instance, uint32_t port, void* data)
{
	Tremolo* tremolo = (Tremolo*) instance;

	switch ((PortIndex) port) {
		case INPUT_L:
			tremolo->input[0] = (const float*) data;
			break;
		case INPUT_R:
			tremolo->input[1] = (const float*) data;
			break;
		case OUTPUT_L:
			tremolo->output[0] = (float*) data;
			break;
		case OUTPUT_R:
			tremolo->output[1] = (float*) data;
			break;
		case SPEED:
			tremolo->speed = (const float*) data;
			break;
		case DEPTH:
			tremolo->depth = (const float*) data;
			break;
	}
}

static void activate(LV2_Handle instance)
{
	Tremolo* tremolo = (Tremolo*) instance;

	tremolo->sweep = 3.141592653589793238 / 2.0;
	tremolo->speedChase = 0.0;
	tremolo->depthChase = 0.0;
	tremolo->speedAmount = 1.0;
	tremolo->depthAmount = 0.0;
	tremolo->lastSpeed = 1000.0;
	tremolo->lastDepth = 1000.0;
	tremolo->fpdL = 1.0;
	while (tremolo->fpdL < 16386) tremolo->fpdL = rand() * UINT32_MAX;
	tremolo->fpdR = 1.0;
	while (tremolo->fpdR < 16386) tremolo->fpdR = rand() * UINT32_MAX;
}

static void run(LV2_Handle instance, uint32_t sampleFrames)
{
	Tremolo* tremolo = (Tremolo*) instance;

	const float* in1 = tremolo->input[0];
	const float* in2 = tremolo->input[1];
	float* out1 = tremolo->output[0];
	float* out2 = tremolo->output[1];

	double overallscale = 1.0;
	overallscale /= 44100.0;
	overallscale *= tremolo->sampleRate;

	tremolo->speedChase = pow(*tremolo->speed, 4);
	tremolo->depthChase = *tremolo->depth;
	double speedSpeed = 300 / (fabs(tremolo->lastSpeed - tremolo->speedChase) + 1.0);
	double depthSpeed = 300 / (fabs(tremolo->lastDepth - tremolo->depthChase) + 1.0);
	tremolo->lastSpeed = tremolo->speedChase;
	tremolo->lastDepth = tremolo->depthChase;

	double speed;
	double depth;
	double skew;
	double density;

	double tupi = 3.141592653589793238;
	double control;
	double tempcontrol;
	double thickness;
	double out;
	double bridgerectifier;
	double offset;

	while (sampleFrames-- > 0) {
		double inputSampleL = *in1;
		double inputSampleR = *in2;
		if (fabs(inputSampleL) < 1.18e-23) inputSampleL = tremolo->fpdL * 1.18e-17;
		if (fabs(inputSampleR) < 1.18e-23) inputSampleR = tremolo->fpdR * 1.18e-17;
		double drySampleL = inputSampleL;
		double drySampleR = inputSampleR;

		tremolo->speedAmount = (((tremolo->speedAmount * speedSpeed) + tremolo->speedChase) / (speedSpeed + 1.0));
		tremolo->depthAmount = (((tremolo->depthAmount * depthSpeed) + tremolo->depthChase) / (depthSpeed + 1.0));
		speed = 0.0001 + (tremolo->speedAmount / 1000.0);
		speed /= overallscale;
		depth = 1.0 - pow(1.0 - tremolo->depthAmount, 5);
		skew = 1.0 + pow(tremolo->depthAmount, 9);
		density = ((1.0 - tremolo->depthAmount) * 2.0) - 1.0;

		offset = sin(tremolo->sweep);
		tremolo->sweep += speed;
		if (tremolo->sweep > tupi) {
			tremolo->sweep -= tupi;
		}
		control = fabs(offset);
		if (density > 0) {
			tempcontrol = sin(control);
			control = (control * (1.0 - density)) + (tempcontrol * density);
		} else {
			tempcontrol = 1 - cos(control);
			control = (control * (1.0 + density)) + (tempcontrol * -density);
		}

		thickness = ((control * 2.0) - 1.0) * skew;
		out = fabs(thickness);

		bridgerectifier = fabs(inputSampleL);
		if (bridgerectifier > 1.57079633) bridgerectifier = 1.57079633;
		if (thickness > 0) bridgerectifier = sin(bridgerectifier);
		else bridgerectifier = 1 - cos(bridgerectifier);
		if (inputSampleL > 0) inputSampleL = (inputSampleL * (1 - out)) + (bridgerectifier * out);
		else inputSampleL = (inputSampleL * (1 - out)) - (bridgerectifier * out);
		inputSampleL *= (1.0 - control);
		inputSampleL *= 2.0;
		inputSampleL = (drySampleL * (1 - depth)) + (inputSampleL * depth);

		bridgerectifier = fabs(inputSampleR);
		if (bridgerectifier > 1.57079633) bridgerectifier = 1.57079633;
		if (thickness > 0) bridgerectifier = sin(bridgerectifier);
		else bridgerectifier = 1 - cos(bridgerectifier);
		if (inputSampleR > 0) inputSampleR = (inputSampleR * (1 - out)) + (bridgerectifier * out);
		else inputSampleR = (inputSampleR * (1 - out)) - (bridgerectifier * out);
		inputSampleR *= (1.0 - control);
		inputSampleR *= 2.0;
		inputSampleR = (drySampleR * (1 - depth)) + (inputSampleR * depth);

		int expon;
		frexpf((float) inputSampleL, &expon);
		tremolo->fpdL ^= tremolo->fpdL << 13;
		tremolo->fpdL ^= tremolo->fpdL >> 17;
		tremolo->fpdL ^= tremolo->fpdL << 5;
		inputSampleL += (((double) tremolo->fpdL - (uint32_t) 0x7fffffff) * 5.5e-36l * pow(2, expon + 62));
		frexpf((float) inputSampleR, &expon);
		tremolo->fpdR ^= tremolo->fpdR << 13;
		tremolo->fpdR ^= tremolo->fpdR >> 17;
		tremolo->fpdR ^= tremolo->fpdR << 5;
		inputSampleR += (((double) tremolo->fpdR - (uint32_t) 0x7fffffff) * 5.5e-36l * pow(2, expon + 62));

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
	TREMOLO_URI,
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
