#include "allocore/io/al_App.hpp"
using namespace al;
using namespace std;

// (0, 1)
struct Phasor {
  float phase = 0, increment = 0.001;
  void frequency(float hz, float sampleRate) { increment = hz / sampleRate; }
  float getNextSample() {
    float returnValue = phase;
    phase += increment;
    if (phase > 1) phase -= 1;
    return returnValue;
  }
  float operator()() { return getNextSample(); }
};

// (-1, 1)
struct Sawtooth : Phasor {
  float getNextSample() { return 2 * Phasor::getNextSample() - 1; }
  float operator()() { return getNextSample(); }
};

struct Reverb {
  float effectSample(float sample) { return 0; }
  float operator()(float sample) { return effectSample(sample); }
};

// Phasor p, s;
// Gain g;
// Reverb r;
// p.getNextSample() + r.effectSample(s.getNextSample()) * gain.getNextSample()
//  overload call operator
// p() + r(s()) * gain()

struct MyApp : App {
  MyApp() {
    initWindow();
    initAudio(44100);
    saw.frequency(261, 44100);
    frequency.frequency(0.7, 44100);
  }

  Sawtooth saw;
  Phasor frequency;

  void onSound(AudioIOData& io) {
    while (io()) {
      saw.frequency(200 + 100 * frequency(), 44100);
      float s = saw() * 0.3;
      io.out(0) = s;
      io.out(1) = s;
    }
  }

  void onKeyDown(const ViewpointWindow&, const Keyboard& k) {
    switch (k.key()) {
      default:
      case '1':
        break;
      case '2':
        break;
      case '3':
        break;
      case '4':
        break;
    }
  }

  // other stuff
  void onAnimate(double dt) {}
  void onDraw(Graphics& g) {}
};

int main() { MyApp().start(); }