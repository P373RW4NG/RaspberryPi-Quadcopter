#ifndef FILTER_H
#define FILTER_H

//complementary filter
float complFtr(float output, float ang_a, float g, float dt, float a){
    output = a*(output + g*dt) + (1-a)*ang_a;
    return output;
}

#endif // FILTER_H
