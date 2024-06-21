/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"

const int AMPLITUDE = 10000;
const int FREQUENCY = 440;  // Frequency of the sound in Hz (A4 note)
const int SAMPLE_RATE = 44100;
const int DURATION = 5000;  // Duration in milliseconds

// Struct to hold the audio data
struct AudioData {
    int phase;
    int phaseIncrement;
    int volume;
};


// Audio callback function to generate audio
void audioCallback(void* userdata, Uint8* stream, int len) {
    AudioData* audio = (AudioData*)userdata;
    Sint16* buffer = (Sint16*)stream;
    int length = len / 2;  // Each sample is 2 bytes (16-bit audio)

    for (int i = 0; i < length; ++i) {
        buffer[i] = (Sint16)(audio->volume * sin(2.0 * M_PI * audio->phase / SAMPLE_RATE));
        audio->phase = (audio->phase + audio->phaseIncrement) % SAMPLE_RATE;
    }
}


Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 0.004, 0.004, 400, 0.005, 0.045, 2 / 2 / M_PI;
    R.row(0) << 50, 7;
    R.row(1) << 7, 50;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}


int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    /**
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed
    */
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;
    std::vector<float> speed_history;
    std::vector<double> historyTime;

    if (SDL_Init(SDL_INIT_AUDIO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
    }

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        time_t start_time = time(0);
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y, t;
        int lastX = 0;
        int lastY = 0;
        double speed = 0;
        double travel = 0;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);
        Eigen::VectorXf stateHistory = Eigen::VectorXf::Zero(6);
        int time_stops = 1;
        int counter = 0;

        auto start = std::chrono::high_resolution_clock::now();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;

        SDL_AudioSpec wantedSpec;
        SDL_zero(wantedSpec);
        wantedSpec.freq = SAMPLE_RATE;
        wantedSpec.format = AUDIO_S16SYS;
        wantedSpec.channels = 1;
        wantedSpec.samples = 4096;
        wantedSpec.callback = audioCallback;

        // Set up the audio data
        AudioData audioData;
        audioData.phase = 0;
        audioData.phaseIncrement = FREQUENCY;
        audioData.volume = AMPLITUDE;

        // Assign the user data to the audio spec
        wantedSpec.userdata = &audioData;

        SDL_AudioDeviceID deviceID = SDL_OpenAudioDevice(NULL, 0, &wantedSpec, NULL, 0);
        if (deviceID == 0) {
            std::cerr << "Failed to open audio device! SDL_Error: " << SDL_GetError() << std::endl;
        }

        while (!quit)
        {
            audioData.phaseIncrement = (int)speed * 10 + 200;

            stateHistory = quadrotor.GetState();

            end = std::chrono::high_resolution_clock::now();
            duration = end - start;
            if (duration.count() >= 10) {
                start = std::chrono::high_resolution_clock::now();
                counter += 1;
                if (counter >= 180) counter = 0;
            }

            if (difftime(time(0), start_time) >= time_stops) {
                // measure drone speed
                travel = sqrt(pow((stateHistory[1] - lastY), 2) + pow((stateHistory[0] - lastX), 2));
                speed = travel;
                lastX = stateHistory[0];
                lastY = stateHistory[1];
                std::cout << "Actual Drone Speed: " << (int)speed << " pps" << std::endl;
                std::cout << "Counter: " << counter << std::endl;

                // 
                x_history.push_back(stateHistory[0]);
                y_history.push_back(stateHistory[1]);
                theta_history.push_back(stateHistory[2] * 180 / M_PI);
                speed_history.push_back(speed);
                historyTime.push_back(time_stops - 1);
                time_stops += 1;

            }


            //events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                /*else if (e.type == SDL_MOUSEMOTION)
                {
                    SDL_GetMouseState(&x, &y);
                    std::cout << "Mouse position: (" << x << ", " << y << ")" << std::endl;
                }*/
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    SDL_GetMouseState(&x, &y);
                    state << x - SCREEN_WIDTH / 2, y - SCREEN_HEIGHT / 2, 0, 0, 0, 0;
                    quadrotor.SetGoal(state);
                }
                else if (e.type == SDL_KEYUP) {
                    if (e.key.keysym.sym == SDLK_p) {


                        matplot::tiledlayout(2, 2);
                        auto ax1 = matplot::nexttile();
                        auto p1 = matplot::plot(ax1, historyTime, x_history, "-o");
                        matplot::xlabel(ax1, "time");
                        matplot::ylabel(ax1, "x");

                        auto ax2 = matplot::nexttile();
                        auto p2 = matplot::plot(ax2, historyTime, y_history, "--xr");
                        matplot::xlabel(ax2, "time");
                        matplot::ylabel(ax2, "y");

                        auto ax3 = matplot::nexttile();
                        auto p3 = matplot::plot(ax3, historyTime, theta_history, "-:gr");
                        matplot::xlabel(ax3, "time");
                        matplot::ylabel(ax3, "theta");

                        auto ax4 = matplot::nexttile();
                        auto p4 = matplot::plot(ax4, historyTime, speed_history, "-*");
                        matplot::xlabel(ax4, "time");
                        matplot::ylabel(ax4, "speed");

                        matplot::show();
                    }
                }

            }
            SDL_PauseAudioDevice(deviceID, 0);
            SDL_Delay((int)dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer, counter);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}