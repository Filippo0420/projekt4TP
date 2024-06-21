#include "planar_quadrotor_visualizer.h"
#include <tuple>
#include <chrono>

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


void drawEmptyElipse(SDL_Renderer* renderer, int x1, int y1, int x2, int y2, double b) {
    double h = (x1 + x2) / 2.0;
    double k = (y1 + y2) / 2.0;

    double a = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)) / 2.0;

    double theta = std::atan2(y2 - y1, x2 - x1);

    // Step through the angle t from 0 to 2π
    for (double t = 0; t <= 2 * M_PI; t += 0.01) {
        double x_prime = a * std::cos(t);
        double y_prime = b * std::sin(t);

        double x = h + x_prime * std::cos(theta) - y_prime * std::sin(theta);
        double y = k + x_prime * std::sin(theta) + y_prime * std::cos(theta);

        SDL_RenderDrawPoint(renderer, static_cast<int>(std::round(x)), static_cast<int>(std::round(y)));
    }
}


std::tuple<int, int> getElipsePoint(int x1, int y1, double theta, double fi, int a, int b) {
    
    double x_prime = a * std::cos(fi);
    double y_prime = b * std::sin(fi);

    double x = x1 + x_prime * std::cos(theta) - y_prime * std::sin(theta);
    double y = y1 + x_prime * std::sin(theta) + y_prime * std::cos(theta);
    return { static_cast<int>(std::round(x)), static_cast<int>(std::round(y)) };
}


void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer, int propAngle) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;
    const int screenX = 1280;
    const int screenY = 720;
    /* x, y, theta coordinates */
    q_x = state[0] + screenX/2;
    q_y = state[1] + screenY/2;
    q_theta = state[2];


    int propWidth = 40;
    int propHeight = 8;
    int propElipseWidth = 70;
    int propElipseHeight = 16;

    int rectW = 50;
    int rectH = 10;
    int rectX = q_x - rectW/2;
    int rectY = q_y - rectH/2;


    int leftSideX = q_x + (rectW / 2) * -cos(q_theta);
    int leftSideY = q_y + (rectW / 2) * -sin(q_theta);
    int rightSideX = q_x + (rectW / 2) * cos(q_theta);
    int rightSideY = q_y + (rectW / 2) * sin(q_theta);

    int leftSideX2 = leftSideX + rectH * -cos(q_theta - 0.5 * M_PI);
    int leftSideY2 = leftSideY + rectH * -sin(q_theta - 0.5 * M_PI);
    int rightSideX2 = rightSideX + rectH * cos(q_theta + 0.5 * M_PI);
    int rightSideY2 = rightSideY + rectH * sin(q_theta + 0.5 * M_PI);

    int leftPropCentreX = leftSideX2 - 2*rectH * cos(q_theta + 0.5 * M_PI);
    int leftPropCentreY = leftSideY2 - 2*rectH * sin(q_theta + 0.5 * M_PI);

    int rightPropCentreX = rightSideX2 - 2 * rectH * cos(q_theta + 0.5 * M_PI);
    int rightPropCentreY = rightSideY2 - 2 * rectH * sin(q_theta + 0.5 * M_PI);

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), leftSideX, leftSideY, rightSideX, rightSideY);
    SDL_RenderDrawLine(gRenderer.get(), leftSideX, leftSideY, leftSideX2, leftSideY2);

    SDL_SetRenderDrawColor(gRenderer.get(), 0x11, 0x00, 0xFF, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), rightSideX, rightSideY, rightSideX2, rightSideY2);
    SDL_RenderDrawLine(gRenderer.get(), leftSideX2, leftSideY2, rightSideX2, rightSideY2);

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), leftSideX2, leftSideY2, leftPropCentreX, leftPropCentreY);
    SDL_RenderDrawLine(gRenderer.get(), rightSideX2, rightSideY2, rightPropCentreX, rightPropCentreY);
    

    SDL_SetRenderDrawColor(gRenderer.get(), 0x11, 0xAA, 0xAA, 0xFF);
    std::tuple<int, int> actualProp1LeftEnd = getElipsePoint(leftPropCentreX, leftPropCentreY, q_theta, M_PI * cos(propAngle * (M_PI / 180)), propElipseWidth / 2, propElipseHeight / 2);
    drawEmptyElipse(gRenderer.get(), leftPropCentreX, leftPropCentreY, std::get<0>(actualProp1LeftEnd), std::get<1>(actualProp1LeftEnd), propHeight / 2);

    std::tuple<int, int> actualProp1RightEnd = getElipsePoint(leftPropCentreX, leftPropCentreY, q_theta, M_PI + M_PI * cos(propAngle * (M_PI / 180)), propElipseWidth / 2, propElipseHeight / 2);
    drawEmptyElipse(gRenderer.get(), leftPropCentreX, leftPropCentreY, std::get<0>(actualProp1RightEnd), std::get<1>(actualProp1RightEnd), propHeight / 2);




    SDL_SetRenderDrawColor(gRenderer.get(), 0xFA, 0xA0, 0xA0, 0xFF);
    std::tuple<int, int> actualProp2LeftEnd = getElipsePoint(rightPropCentreX, rightPropCentreY, q_theta, M_PI * cos(propAngle * (M_PI / 180)), propElipseWidth / 2, propElipseHeight / 2);
    drawEmptyElipse(gRenderer.get(), rightPropCentreX, rightPropCentreY, std::get<0>(actualProp2LeftEnd), std::get<1>(actualProp2LeftEnd), propHeight / 2);

    std::tuple<int, int> actualProp2RightEnd = getElipsePoint(rightPropCentreX, rightPropCentreY, q_theta, M_PI + M_PI * cos(propAngle * (M_PI / 180)), propElipseWidth / 2, propElipseHeight / 2);
    drawEmptyElipse(gRenderer.get(), rightPropCentreX, rightPropCentreY, std::get<0>(actualProp2RightEnd), std::get<1>(actualProp2RightEnd), propHeight / 2);
}

