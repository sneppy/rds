#pragma once

#include "coremin.h"
#include <SDL.h>

namespace InputMappings
{
	enum Type : int32
	{
		NONE		= -1,
		KEY_ESC		= SDLK_ESCAPE,
		KEY_SPACE	= SDLK_SPACE,
		KEY_A		= SDLK_a,
		KEY_B		= SDLK_b,
		KEY_D		= SDLK_d,
		KEY_S		= SDLK_s,
		KEY_W		= SDLK_w,
		KEY_RIGHT	= SDLK_RIGHT,
		KEY_LEFT	= SDLK_LEFT,
		KEY_DOWN	= SDLK_DOWN,
		KEY_UP		= SDLK_UP,
		KEY_ENTER	= SDLK_KP_ENTER
	};
} // Input Bindings

struct Axis
{
public:
	/// Positive binding
	InputMappings::Type positive;

	/// Negative binding
	InputMappings::Type negative;

	/// Axis multiplier
	float32 multiplier;
};

class InputManager : public Singleton<InputManager>
{
public:
	using AxisDelegateT = void (*)(float32);

protected:
	/// Map of key states
	Map<InputMappings::Type, int32> keyStates;

	/// Map of registered axes
	Map<String, Axis> axes;

	/// Array of registered axis bindings
	Array<Pair<String, AxisDelegateT>> axisBindings;

public:
	FORCE_INLINE void createAxis(const String & name, InputMappings::Type positive, InputMappings::Type negative, float32 multiplier = 1.f)
	{
		axes[name] = Axis{positive, negative, multiplier};

		/// Create entries for negative and positive bindings
		keyStates[positive],
		keyStates[negative];
	}

	FORCE_INLINE void addAxisBinding(const String & axis, AxisDelegateT action)
	{
		if (axes.find(axis) != axes.end())
			axisBindings.push(makePair(axis, action));
	}

	FORCE_INLINE void handleEvent(SDL_Event * event)
	{
		switch (event->type)
		{
			case SDL_KEYDOWN:
			{
				keyStates[(InputMappings::Type)event->key.keysym.sym] = 1;
				break;
			}
			case SDL_KEYUP:
			{
				keyStates[(InputMappings::Type)event->key.keysym.sym] = 0;
				break;
			}
		}
	}

	FORCE_INLINE void tickInput(float32 dt)
	{
		for (const auto & axisBinding : axisBindings)
		{
			const auto axis = (*axes.find(axisBinding.first)).second;
			
			if (axis.negative == InputMappings::NONE)
			{
				const int32
					positiveVal = (*keyStates.find(axis.positive)).second;
				
				const float32 val = positiveVal * axis.multiplier;

				(axisBinding.second)(val);
			}
			else
			{
				const int32
					positiveVal = (*keyStates.find(axis.positive)).second,
					negativeVal = (*keyStates.find(axis.negative)).second;
				
				const float32 val = (positiveVal - negativeVal) * axis.multiplier;

				(axisBinding.second)(val);
			}
		}
	}
};