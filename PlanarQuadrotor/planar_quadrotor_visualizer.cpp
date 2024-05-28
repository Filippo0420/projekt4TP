#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */

void drawEllipse(SDL_Renderer* renderer, int x0, int y0, int width, int height) {
    int a = width / 2;
    int b = height / 2;
    int centerX = x0 + a;
    int centerY = y0 + b;
    int dx = 0;
    int dy = b;
    long a2 = a * a;
    long b2 = b * b;
    long err = b2 - (2 * b - 1) * a2, e2; /* error of 1.step */

    do {
        SDL_RenderDrawPoint(renderer, centerX + dx, centerY + dy);
        SDL_RenderDrawPoint(renderer, centerX - dx, centerY + dy);
        SDL_RenderDrawPoint(renderer, centerX - dx, centerY - dy);
        SDL_RenderDrawPoint(renderer, centerX + dx, centerY - dy);

        e2 = 2 * err;
        if (e2 < (2 * dx + 1) * b2) { err += (2 * dx + 1) * b2; ++dx; }
        if (e2 > -(2 * dy - 1) * a2) { err -= (2 * dy - 1) * a2; --dy; }
    } while (dy >= 0);

    while (dx++ < a) {
        SDL_RenderDrawPoint(renderer, centerX + dx, centerY);
        SDL_RenderDrawPoint(renderer, centerX - dx, centerY);
    }
}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;
    const int screenX = 1280;
    const int screenY = 720;
    /* x, y, theta coordinates */
    q_x = state[0] + screenX/2;
    q_y = state[1] + screenY/2;
    q_theta = state[2];

    SDL_Rect rect;
    rect.w = 50;
    rect.h = 10;
    rect.x = q_x - rect.w/2;
    rect.y = q_y - rect.h/2;

    int propWidth = 30;
    int propHeight = 10;

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    //filledCircleColor(gRenderer.get(), q_x, q_y, 30, 0xFFAA00FF);

    /* drawing drone */
    SDL_RenderFillRect(gRenderer.get(), &rect);
    drawEllipse(gRenderer.get(), rect.x - propWidth / 2, rect.y - rect.h, propWidth / 2, propHeight);
    drawEllipse(gRenderer.get(), rect.x, rect.y - rect.h, propWidth / 2, propHeight);
    drawEllipse(gRenderer.get(), rect.x + rect.w - propWidth/2, rect.y - rect.h, propWidth / 2, propHeight);
    drawEllipse(gRenderer.get(), rect.x + rect.w, rect.y - rect.h, propWidth / 2, propHeight);

    
}

