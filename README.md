# picoscope-2406B-DAQ-VS
Visual Studio Community 2022 project to control the picoscope 2406B digitizer with a PC. This code was used for coincidence (waveform) acquisition between CH1 and CH2 (rising edges). It is possible to use it for acquisition of individual channels if channel trigger conditions are changed

Requirements:

  1. PicoSDK for 2406B model and your corresponding platform (found in https://www.picotech.com/downloads).

IMPORTANT:

  1. For the code to run, you need to create a folder "data" in the source file directory.

CONFIGURATION:

  1. Variable to change trigger channel polarity: TRIGGER_DIRECTIONS directions
  2. Variable to change trigger logic: PS2000A_TRIGGER_CONDITIONS conditions
  3. Variable to activate other channels: int16_t unit->channelSettings[channelNumber].enabled = TRUE
  4. Variable to change channel voltage range: int16_t unit->channelSettings[channelNumber].range
  5. Variable to change channel analog offset: float unit->channelSettings[channelNumber].analogOffset
  6. Variable to change trigger voltage level: float lowTrigger/highTrigger (depending on trigger hysteresis)
  7. Variable to change number of events to acquire: int numSegments
  8. Variable to change number of samples per waveform event (sampling rate with 2 channels enabled is 500 MS/s, which gives a sampling time of 2 ns): int numSamplesPerSegment

EVENT SAVING FORMAT:

Saving format is binary. Each event is saved in an interleaved way. This means that CH1's first sample (1B) is followed by CH2's first sample (1B), followed by CH1's second sample (1B), followed by CH2's second sample (1B), and so on. The waveform file size in Bytes is 2 * numSegments * numSamplesPerSegment
