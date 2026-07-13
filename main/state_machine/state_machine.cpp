#include <stdio.h>
#include <stdint.h>
#include "command.h"

enum class SystemState {
    Inactive,
    Active,
    Help
};

class StateMachine {
public:
    void update(Command command);

private:
    SystemState state = SystemState::Inactive;
};

void StateMachine::update(Command command)
{
    // Global emergency/help command
    if(command == Command::ArmHelp)
    {
        // motor.release();
        state = SystemState::Help;
        return;
    }

    switch(state)
    {
        case SystemState::Inactive:

            if(command == Command::ArmUp)
            {
                // motor.moveUp();
                state = SystemState::Active;
            }

            else if(command == Command::ArmDown)
            {
                // motor.moveDown();
                state = SystemState::Active;
            }

            break;


        case SystemState::Active:

            if(command == Command::Up)
            {
                // motor.moveUp();
            }

            else if(command == Command::Down)
            {
                // motor.moveDown();
            }

            else if(command == Command::Stop)
            {
                // motor.stop();
                state = SystemState::Inactive;
            }

            break;


        case SystemState::Help:

            // Ignore all voice commands
            // until hardware reset

            break;
    }
}