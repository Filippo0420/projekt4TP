#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */

void drawFilledEllipse(SDL_Renderer* renderer, int centerX, int centerY, int radiusX, int radiusY) {
    for (int y = -radiusY; y <= radiusY; y++) {
        for (int x = -radiusX; x <= radiusX; x++) {
            if ((x * x * radiusY * radiusY + y * y * radiusX * radiusX) <= (radiusX * radiusX * radiusY * radiusY)) {
                SDL_RenderDrawPoint(renderer, centerX + x, centerY + y);
            }
        }
    }
}

// Function to draw and fill a rotated rectangle with an additional shape (ellipse) inside
void drawRotatedRect(SDL_Renderer* renderer, int x, int y, int w, int h, double angle) {
    // Create a texture to represent the rectangle
    SDL_Texture* rectTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, w, h);

    // Set the texture as the rendering target
    SDL_SetRenderTarget(renderer, rectTexture);

    // Fill the texture with red color for the rectangle
    SDL_SetRenderDrawColor(renderer, 0xFF, 0x00, 0x00, 0xFF);
    SDL_Rect fillRect = { 0, 0, w, h };
    SDL_RenderFillRect(renderer, &fillRect);

    // Draw a filled blue ellipse inside the rectangle
    SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0xFF, 0xFF);
    drawFilledEllipse(renderer, x - w/2, y - h/2, w/4, h/4);

    // Reset the rendering target to the default
    SDL_SetRenderTarget(renderer, NULL);

    // Define the destination rectangle on the screen
    SDL_Rect destRect = { x, y, w, h};

    // Copy the texture to the renderer with rotation
    SDL_RenderCopyEx(renderer, rectTexture, NULL, &destRect, angle, NULL, SDL_FLIP_NONE);

    // Destroy the texture
    SDL_DestroyTexture(rectTexture);
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

    int rectW = 50;
    int rectH = 10;
    int rectX = q_x - rectW/2;
    int rectY = q_y - rectH/2;

    rect.w = rectW;
    rect.h = rectH;
    rect.x = rectX;
    rect.y = rectY;

    int leftSideX = q_x + (rectW / 2) * -cos(q_theta);
    int leftSideY = q_y + (rectW / 2) * -sin(q_theta);
    int rightSideX = q_x + (rectW / 2) * cos(q_theta);
    int rightSideY = q_y + (rectW / 2) * sin(q_theta);

    int leftSideX2 = leftSideX + rectH * -cos(q_theta - 0.5 * M_PI);
    int leftSideY2 = leftSideY + rectH * -sin(q_theta - 0.5 * M_PI);
    int rightSideX2 = rightSideX + rectH * cos(q_theta + 0.5 * M_PI);
    int rightSideY2 = rightSideY + rectH * sin(q_theta + 0.5 * M_PI);

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    //filledCircleColor(gRenderer.get(), q_x, q_y, 30, 0xFFAA00FF);

    /* drawing drone */
    //drawRotatedRect(gRenderer.get(), rectX, rectY, rectW, rectH, q_theta);
    //SDL_RenderFillRect(gRenderer.get(), &rect);
    SDL_RenderDrawLine(gRenderer.get(), leftSideX, leftSideY, rightSideX, rightSideY);
    SDL_RenderDrawLine(gRenderer.get(), leftSideX, leftSideY, leftSideX2, leftSideY2);

    SDL_SetRenderDrawColor(gRenderer.get(), 0x11, 0x00, 0xFF, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), rightSideX, rightSideY, rightSideX2, rightSideY2);
    SDL_RenderDrawLine(gRenderer.get(), leftSideX2, leftSideY2, rightSideX2, rightSideY2);

    SDL_SetRenderDrawColor(gRenderer.get(), 0xAA, 0x00, 0xAA, 0xFF);
    drawFilledEllipse(gRenderer.get(), leftSideX, leftSideY - rectH, rectW / 4, rectH / 2);

    SDL_SetRenderDrawColor(gRenderer.get(), 0xAA, 0x00, 0xAA, 0xFF);
    drawFilledEllipse(gRenderer.get(), rightSideX, rightSideY - rectH, rectW / 4, rectH / 2);
    
}

