/*
    performer.h
    Plays back a sequence of steps.
*/

/*----------------------------------------------------------*\
| Structures
\*----------------------------------------------------------*/

struct Step
{
  float time;
  float duration;
  float command;
};

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

class performer
{
public:
  // Steps to play back
  Step* steps;

  // How many steps
  int stepCount;

  // Current sequence time
  float elapsed;

  // Sequence duration
  float total;

  // Current step
  int step;

  // Whether to loop or stop when done
  bool loop;

  // Time playback started or looped
  ros::Time start;

  // Whether playback is complete
  bool done;

public:
  performer():
    steps(nullptr),
    stepCount(0),
    elapsed(0),
    total(0),
    step(0),
    loop(false),
    done(false)
  {
  }

public:
  double getCommand(ros::Time time)
  {
    if (!stepCount || done) return 0.0;

    elapsed = (time - start).toSec();

    if (elapsed >= total)
    {
      if (loop)
      {
        // Loop playback
        start = time;
        step = 0;
      }
      else
      {
        // Stop playback
        done = true;
        return 0.0;
      }
    }
    else if (elapsed >= steps[step].time + steps[step].duration)
    {
      // Continue playback
      step++;
    }

    return steps[step].command;
  }

  void play(ros::Time time, const pidtuner::StepCommand& msg)
  {
    stepCount = 0;
    step = 0;
    total = 0.0;

    if (msg.steps_length)
    {
      Step* oldSteps = steps;
      Step* newSteps = new Step[msg.steps_length];

      for (int n = 0; n < msg.steps_length; n++)
      {
        newSteps[n].time = total;
        newSteps[n].duration = msg.steps[n].duration;
        newSteps[n].command = msg.steps[n].command;
        total = total + msg.steps[n].duration;
      }

      steps = newSteps;
      stepCount = msg.steps_length;
      loop = msg.loop;
      done = false;
      start = time;

      delete[] oldSteps;
    }
  }
};
