#include "state_machine.h"

#include "Config.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"


static const char* TAG="StateMachine";



StateMachine::StateMachine(MotionController& motion)
    : motion_(motion)
{

    esp_timer_create_args_t args = {

        .callback = &StateMachine::onTimeout,

        .arg = this,

        .dispatch_method = ESP_TIMER_TASK,

        .name = "smTimeout",

        .skip_unhandled_events = true
    };


    esp_err_t err =
        esp_timer_create(&args,&timeoutTimer);


    if(err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Timer creation failed");
    }
}



StateMachine::~StateMachine()
{
    if(timeoutTimer)
    {
        esp_timer_stop(timeoutTimer);
        esp_timer_delete(timeoutTimer);
    }
}



void StateMachine::armTimer(uint64_t timeoutUs)
{
    esp_timer_stop(timeoutTimer);

    esp_timer_start_once(
        timeoutTimer,
        timeoutUs
    );
}



void StateMachine::disarmTimer()
{
    esp_timer_stop(timeoutTimer);
}



void StateMachine::onTimeout(void* arg)
{
    static_cast<StateMachine*>(arg)
        ->handleTimeout();
}



void StateMachine::handleTimeout()
{

    ESP_LOGW(TAG,
             "Inactive timeout");


    state = SystemState::Inactive;

    stopped=false;


    motion_.stop();
}



Command StateMachine::labelToCommand(const char* label)
{

    if(strcmp(label,"activate")==0)
        return Command::Activate;


    if(strcmp(label,"up")==0)
        return Command::Up;


    if(strcmp(label,"down")==0)
        return Command::Down;


    if(strcmp(label,"stop")==0)
        return Command::Stop;


    if(strcmp(label,"help")==0)
        return Command::Help;


    return Command::None;
}



void StateMachine::determineCommand(ei_impulse_result_t res)
{

    int bestIndex=-1;

    float bestValue=0;



    for(size_t i=0;i<EI_CLASSIFIER_LABEL_COUNT;i++)
    {

        float value =
            res.classification[i].value;


        if(value > bestValue)
        {
            bestValue=value;
            bestIndex=i;
        }
    }



    if(bestIndex < 0 ||
       bestValue < kConfidenceThreshold)
    {
        return;
    }



    Command cmd =
        labelToCommand(
            res.classification[bestIndex].label
        );



    if(cmd == Command::None)
        return;



    uint32_t now =
        esp_timer_get_time()/1000;



    // cooldown
    if(now-lastCommandMs < kCommandCooldownMs)
    {
        ESP_LOGI(TAG,
                 "Command ignored (cooldown)");
        return;
    }



    lastCommandMs=now;



    ESP_LOGI(TAG,
             "Detected %s %.2f",
             commandToStr(cmd),
             bestValue);



    update(cmd);

}




void StateMachine::update(Command command)
{

    switch(command)
    {


    case Command::Activate:

        if(state == SystemState::Inactive)
        {

            ESP_LOGI(TAG,
                     "Activated");


            state =
                SystemState::Active;


            stopped=false;


            armTimer(kInactivityTimeoutUs);
        }

        break;



    case Command::Up:


        if(state != SystemState::Active)
            break;



        // already moving up
        if(motion_.getState()
           == MotionState::MOVING_UP)
        {

            ESP_LOGI(TAG,
                     "Already moving up");

            break;
        }



        ESP_LOGI(TAG,
                 "Moving up");


        motion_.moveUp(
            Config::kDefaultErpm
        );


        stopped=false;


        armTimer(kInactivityTimeoutUs);


        break;




    case Command::Down:


        if(state != SystemState::Active)
            break;



        // already moving down
        if(motion_.getState()
           == MotionState::MOVING_DOWN)
        {

            ESP_LOGI(TAG,
                     "Already moving down");

            break;
        }



        ESP_LOGI(TAG,
                 "Moving down");



        motion_.moveDown(
            Config::kDefaultErpm
        );


        stopped=false;


        armTimer(kInactivityTimeoutUs);


        break;




    case Command::Stop:


        if(state == SystemState::Active)
        {

            ESP_LOGI(TAG,
                     "Stopping");


            motion_.stop();


            stopped=true;


            armTimer(kStopTimeoutUs);
        }

        break;




    case Command::Help:


        ESP_LOGW(TAG,
                 "Help command");


        motion_.stop();


        state =
            SystemState::Inactive;


        stopped=false;


        disarmTimer();


        break;



    default:

        break;
    }

}




const char* StateMachine::stateToStr(SystemState s)
{
    switch(s)
    {
        case SystemState::Inactive:
            return "Inactive";

        case SystemState::Active:
            return "Active";
    }

    return "Unknown";
}




const char* StateMachine::commandToStr(Command c)
{

    switch(c)
    {

        case Command::Activate:
            return "Activate";

        case Command::Up:
            return "Up";

        case Command::Down:
            return "Down";

        case Command::Stop:
            return "Stop";

        case Command::Help:
            return "Help";

        default:
            return "None";
    }

}